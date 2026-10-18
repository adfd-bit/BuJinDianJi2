#include "n10_radar.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

UART_HandleTypeDef huart1;
uart_ring_buf_t uart1_rx_buf;
static uint32_t tick;
static unsigned checks;

uint32_t HAL_GetTick(void) { return tick; }

static void reset(void)
{
    tick = 0U;
    ring_buf_init(&uart1_rx_buf);
    n10p_radar_init();
}

static void be16(uint8_t *dest, uint16_t value)
{
    dest[0] = (uint8_t)(value >> 8);
    dest[1] = (uint8_t)value;
}

static void le16(uint8_t *dest, uint16_t value)
{
    dest[0] = (uint8_t)value;
    dest[1] = (uint8_t)(value >> 8);
}

static uint8_t ld06_crc_bitwise(const uint8_t *raw, unsigned length)
{
    uint8_t crc = 0U;
    for (unsigned i = 0U; i < length; ++i) {
        crc ^= raw[i];
        for (unsigned bit = 0U; bit < 8U; ++bit) {
            unsigned shifted = (unsigned)crc << 1;
            crc = (uint8_t)((crc & 0x80U) ? shifted ^ 0x4DU : shifted);
        }
    }
    return crc;
}

static void ld06_frame(uint8_t *raw, unsigned start_centi, unsigned span_centi, const uint16_t *distances)
{
    memset(raw, 0, LD06_FRAME_LEN);
    raw[0] = 0x54U;
    raw[1] = 0x2CU;
    le16(raw + 2, 3567U);
    le16(raw + 4, (uint16_t)(start_centi % 36000U));
    for (unsigned i = 0U; i < LD06_POINTS_PER_FRAME; ++i) {
        le16(raw + 6U + 3U * i, distances[i]);
        raw[8U + 3U * i] = 100U;
    }
    le16(raw + 42, (uint16_t)((start_centi + span_centi) % 36000U));
    le16(raw + 44, (uint16_t)tick);
    raw[46] = ld06_crc_bitwise(raw, 46U);
}

static void frame(uint8_t *raw, unsigned start_centi, unsigned span_centi, const uint16_t *distances)
{
    memset(raw, 0, N10P_FRAME_LEN);
    raw[0] = 0xA5U;
    raw[1] = 0x5AU;
    raw[2] = 0x6CU;
    be16(raw + 3, 600U);
    be16(raw + 5, (uint16_t)(start_centi % 36000U));
    for (unsigned i = 0U; i < N10P_POINTS_PER_FRAME; ++i) {
        be16(raw + 7U + 3U * i, distances[i]);
        raw[9U + 3U * i] = 20U;
    }
    be16(raw + 105, (uint16_t)((start_centi + span_centi) % 36000U));
    for (unsigned i = 0U; i < N10P_FRAME_LEN - 1U; ++i) raw[107] += raw[i];
}

static void push(const uint8_t *data, unsigned count)
{
    for (unsigned i = 0U; i < count; ++i) ring_buf_put(&uart1_rx_buf, data[i]);
    n10p_radar_process();
}

typedef uint16_t (*distance_fn)(unsigned angle, unsigned turn);

static void sweep(unsigned start_degree, unsigned packets, distance_fn get_distance)
{
    for (unsigned packet = 0U; packet < packets; ++packet) {
        unsigned start = start_degree + packet * 32U;
        uint16_t distances[32];
        uint8_t raw[108];
        for (unsigned i = 0U; i < 32U; ++i) {
            unsigned angle = start + i;
            distances[i] = get_distance(angle % 360U, angle / 360U);
        }
        frame(raw, start * 100U, 3200U, distances);
        tick++;
        push(raw, 108U);
    }
}

static n10p_obstacle_snapshot_t get_snapshot(void)
{
    n10p_obstacle_snapshot_t result;
    n10p_get_obstacle_snapshot(&result);
    checks++;
    return result;
}

static uint16_t front_object(unsigned angle, unsigned turn)
{
    (void)turn;
    return angle <= 2U ? 300U : 0U;
}

static uint16_t empty_scan(unsigned angle, unsigned turn)
{
    (void)angle; (void)turn;
    return 0U;
}

static uint16_t changing_seam(unsigned angle, unsigned turn)
{
    if (angle > 2U && angle < 358U) return 0U;
    return turn == 1U ? 1000U : 2000U;
}

static uint16_t two_plus_two_seam(unsigned angle, unsigned turn)
{
    (void)turn;
    return angle == 0U || angle == 1U || angle == 358U || angle == 359U ? 1000U : 0U;
}

static uint16_t ten_objects(unsigned angle, unsigned turn)
{
    (void)turn;
    unsigned object = angle / 30U;
    return object < 10U && angle % 30U < 3U ? (uint16_t)(3000U - object * 200U) : 0U;
}

static uint16_t radial_split(unsigned angle, unsigned turn)
{
    (void)turn;
    if (angle >= 50U && angle <= 52U) return 500U;
    if (angle >= 53U && angle <= 55U) return 1000U;
    return 0U;
}

static uint16_t small_group_and_gap(unsigned angle, unsigned turn)
{
    (void)turn;
    if (angle == 50U || angle == 51U) return 500U;
    if (angle >= 80U && angle <= 82U) return 500U;
    if (angle >= 86U && angle <= 88U) return 500U;
    return 0U;
}

static void test_complete_scan_and_coordinates(void)
{
    reset();
    n10p_obstacle_snapshot_t s = get_snapshot();
    assert(!s.valid && s.age_ms == UINT32_MAX && s.front_mm == UINT32_MAX);
    sweep(350U, 1U, front_object);
    s = get_snapshot();
    assert(!s.valid && s.scan_seq == 0U);
    sweep(382U, 11U, front_object);
    s = get_snapshot();
    assert(s.valid && s.scan_seq == 1U && s.count == 1U);
    assert(s.front_known && s.blocked && s.front_mm >= 419U && s.front_mm <= 423U);
    assert(s.obstacles[0].x_mm >= 110 && s.obstacles[0].x_mm <= 113);
    assert(s.obstacles[0].y_mm >= 405 && s.obstacles[0].y_mm <= 406);
    assert(n10p_radar_valid_frames() == 12U && n10p_radar_bad_frames() == 0U);
}

static void test_crossing_packet_belongs_to_two_scans(void)
{
    reset();
    sweep(350U, 12U, changing_seam);
    n10p_obstacle_snapshot_t s = get_snapshot();
    assert(s.valid && s.count == 1U);
    assert(s.obstacles[0].y_mm >= 1104 && s.obstacles[0].y_mm <= 1106);
    assert(s.obstacles[0].x_mm >= 105 && s.obstacles[0].x_mm <= 108);
}

static void test_seam_merge_before_sample_filter(void)
{
    reset();
    sweep(350U, 12U, two_plus_two_seam);
    n10p_obstacle_snapshot_t s = get_snapshot();
    assert(s.valid && s.count == 1U);
    assert(s.obstacles[0].x_mm >= 96 && s.obstacles[0].x_mm <= 99);
}

static void test_raw_sample_centroid(void)
{
    reset();
    uint16_t distances[32] = {0};
    uint8_t raw[108];
    frame(raw, 35900U, 100U, distances);
    push(raw, 108U);
    frame(raw, 0U, 100U, distances);
    push(raw, 108U);
    distances[0] = 800U; distances[1] = 700U; distances[2] = 650U;
    frame(raw, 1000U, 100U, distances);
    push(raw, 108U);
    sweep(11U, 11U, empty_scan);
    n10p_obstacle_snapshot_t s = get_snapshot();
    assert(s.valid && s.count == 1U);
    /* Three nearby raw returns use their actual center, even within one degree. */
    assert(s.obstacles[0].x_mm >= 230 && s.obstacles[0].x_mm <= 232);
    assert(s.obstacles[0].y_mm >= 811 && s.obstacles[0].y_mm <= 813);
}

static void test_nearest_eight_and_distance_split(void)
{
    reset();
    sweep(350U, 12U, ten_objects);
    n10p_obstacle_snapshot_t s = get_snapshot();
    assert(s.valid && s.count == 8U);
    int64_t previous = -1;
    for (unsigned i = 0U; i < s.count; ++i) {
        int64_t distance = (int64_t)s.obstacles[i].x_mm * s.obstacles[i].x_mm +
                           (int64_t)s.obstacles[i].y_mm * s.obstacles[i].y_mm;
        assert(distance >= previous && distance < 8000000);
        previous = distance;
    }
    reset();
    sweep(350U, 12U, radial_split);
    s = get_snapshot();
    assert(s.valid && s.count == 2U);
    reset();
    sweep(350U, 12U, small_group_and_gap);
    s = get_snapshot();
    assert(s.valid && s.count == 2U);
}

static void test_front_uses_every_raw_sample(void)
{
    reset();
    uint16_t distances[32] = {0};
    uint8_t raw[108];
    frame(raw, 35900U, 100U, distances);
    push(raw, 108U);
    /* Use complete angular progression while placing all returns in one bin. */
    for (unsigned angle = 0U; angle < 360U; ++angle) {
        memset(distances, 0, sizeof(distances));
        if (angle == 340U) {
            distances[0] = 50U;
            distances[1] = 200U;
            distances[2] = 500U;
        }
        frame(raw, angle * 100U, 100U, distances);
        tick++;
        push(raw, 108U);
    }
    memset(distances, 0, sizeof(distances));
    frame(raw, 0U, 100U, distances);
    push(raw, 108U);
    n10p_obstacle_snapshot_t s = get_snapshot();
    assert(s.valid && s.count == 0U && s.front_known && s.blocked);
    assert(s.front_mm >= 295U && s.front_mm <= 298U);
}

static void test_initialization_resets_partial_header(void)
{
    reset();
    const uint8_t header[] = {0x12U, 0xA5U, 0xA5U, 0x5AU};
    push(header, sizeof(header));
    n10p_radar_init();
    sweep(350U, 12U, front_object);
    n10p_obstacle_snapshot_t s = get_snapshot();
    assert(s.valid && n10p_radar_valid_frames() == 12U && n10p_radar_bad_frames() == 0U);
}

static void test_backward_angle_invalidates_scan(void)
{
    reset();
    sweep(350U, 12U, front_object);
    uint16_t distances[32] = {0};
    uint8_t raw[108];
    /* Last point was 13 degrees; this packet goes backwards without crossing zero. */
    frame(raw, 1000U, 100U, distances);
    push(raw, 108U);
    assert(!get_snapshot().valid);
    sweep(350U, 12U, front_object);
    assert(get_snapshot().valid);
}

static void test_timeout_and_recovery(void)
{
    reset();
    sweep(350U, 12U, front_object);
    n10p_obstacle_snapshot_t s = get_snapshot();
    tick += 499U;
    s = get_snapshot();
    assert(s.valid && s.age_ms == 499U);
    tick++;
    s = get_snapshot();
    assert(!s.valid && s.count == 0U && !s.front_known && !s.blocked && s.age_ms == 500U);
    for (unsigned i = 0U; i < 5U; ++i) {
        tick += 10U;
        n10p_radar_process();
    }
    sweep(350U, 12U, front_object);
    s = get_snapshot();
    assert(s.valid && s.scan_seq == 2U);
}

static void test_transport_failures_and_parser_reset(void)
{
    reset();
    sweep(350U, 12U, front_object);
    uint8_t raw[108];
    uint16_t distances[32] = {0};
    frame(raw, 1400U, 3200U, distances);
    raw[107] ^= 1U;
    push(raw, 108U);
    n10p_obstacle_snapshot_t s = get_snapshot();
    assert(!s.valid && s.count == 0U && n10p_radar_bad_frames() == 1U);
    sweep(350U, 12U, front_object);
    s = get_snapshot();
    assert(s.valid && s.scan_seq == 2U);
    uart1_rx_buf.dropped++;
    n10p_radar_process();
    assert(!get_snapshot().valid);
    sweep(350U, 12U, front_object);
    assert(get_snapshot().valid);
    for (unsigned i = 0U; i < UART_RX_BUF_SIZE + 20U; ++i) ring_buf_put(&uart1_rx_buf, 0U);
    n10p_radar_process();
    assert(!get_snapshot().valid && ring_buf_empty(&uart1_rx_buf) && uart1_rx_buf.dropped > 1U);
    sweep(350U, 12U, front_object);
    assert(get_snapshot().valid);
    push(raw, 25U);
    n10p_radar_notify_rx_error();
    assert(!get_snapshot().valid && ring_buf_empty(&uart1_rx_buf));
    sweep(350U, 12U, front_object);
    assert(get_snapshot().valid);
    push(raw, 40U);
    n10p_radar_init();
    sweep(350U, 12U, front_object);
    assert(get_snapshot().valid && n10p_radar_bad_frames() == 0U);
}

static void test_empty_scan_and_tick_wrap(void)
{
    reset();
    tick = UINT32_MAX - 20U;
    sweep(350U, 12U, empty_scan);
    n10p_obstacle_snapshot_t s = get_snapshot();
    assert(s.valid && s.count == 0U && !s.front_known && !s.blocked);
    tick += 499U;
    assert(get_snapshot().valid);
    tick++;
    assert(!get_snapshot().valid);
}

static void test_invalid_endpoints_and_arbitrary_chunks(void)
{
    reset();
    uint16_t distances[32] = {0};
    uint8_t raw[108];
    frame(raw, 1000U, 3200U, distances);
    be16(raw + 5, 40000U);
    raw[107] = 0U;
    for (unsigned i = 0U; i < 107U; ++i) raw[107] += raw[i];
    push(raw, 108U);
    assert(n10p_radar_bad_frames() == 1U && !get_snapshot().valid);
    frame(raw, 35000U, 3200U, distances);
    for (unsigned offset = 0U; offset < 108U; offset += 7U) {
        unsigned length = 108U - offset < 7U ? 108U - offset : 7U;
        push(raw + offset, length);
    }
    assert(n10p_radar_valid_frames() == 1U && !get_snapshot().valid);
    sweep(382U, 11U, empty_scan);
    assert(get_snapshot().valid);
    reset();
    frame(raw, 35900U, 100U, distances);
    be16(raw + 105, 36000U);
    raw[107] = 0U;
    for (unsigned i = 0U; i < 107U; ++i) raw[107] += raw[i];
    push(raw, 108U);
    assert(n10p_radar_valid_frames() == 1U && n10p_radar_bad_frames() == 0U);
}

static void sync_at_zero(void)
{
    uint16_t distances[32] = {0};
    uint8_t raw[108];
    frame(raw, 35900U, 100U, distances);
    push(raw, 108U);
    frame(raw, 0U, 0U, distances);
    push(raw, 108U);
}

static void test_same_bin_radial_jump_is_not_an_object(void)
{
    reset();
    sync_at_zero();
    uint16_t distances[32] = {0};
    uint8_t raw[108];
    distances[0] = 500U; distances[1] = 3000U; distances[2] = 3000U;
    frame(raw, 1000U, 100U, distances);
    push(raw, 108U);
    sweep(11U, 11U, empty_scan);
    n10p_obstacle_snapshot_t s = get_snapshot();
    assert(s.valid && s.count == 0U);
}

static void test_fractional_angle_gap_splits_raw_groups(void)
{
    reset();
    sync_at_zero();
    uint16_t distances[32] = {0};
    uint8_t raw[108];
    distances[0] = distances[1] = 500U;
    frame(raw, 1U, 0U, distances);
    push(raw, 108U);
    frame(raw, 399U, 0U, distances);
    push(raw, 108U);
    sweep(4U, 12U, empty_scan);
    n10p_obstacle_snapshot_t s = get_snapshot();
    assert(s.valid && s.count == 0U);
}

static void test_corrupt_header_invalidates_until_a_new_complete_scan(void)
{
    for (unsigned header_byte = 0U; header_byte < 3U; ++header_byte) {
        reset();
        sweep(350U, 12U, front_object);
        uint16_t distances[32] = {0};
        uint8_t raw[108];
        frame(raw, 1400U, 3200U, distances);
        raw[header_byte] = 0U;
        push(raw, 108U);
        n10p_obstacle_snapshot_t s = get_snapshot();
        assert(!s.valid && s.count == 0U && n10p_radar_bad_frames() == 1U);
        sweep(350U, 1U, front_object);
        assert(!get_snapshot().valid);
        sweep(382U, 11U, front_object);
        s = get_snapshot();
        assert(s.valid && s.scan_seq == 2U);
    }
}

static void test_building_does_not_mutate_published_snapshot(void)
{
    reset();
    sweep(350U, 12U, front_object);
    n10p_obstacle_snapshot_t before = get_snapshot();
    sweep(734U, 8U, ten_objects);
    n10p_obstacle_snapshot_t during = get_snapshot();
    assert(during.valid && during.scan_seq == before.scan_seq && during.count == before.count);
    assert(memcmp(during.obstacles, before.obstacles, sizeof(before.obstacles)) == 0);
    assert(during.front_known == before.front_known && during.front_mm == before.front_mm &&
           during.blocked == before.blocked);
}

static void ld06_sweep(unsigned start_degree, unsigned packets, distance_fn get_distance)
{
    for (unsigned packet = 0U; packet < packets; ++packet) {
        unsigned start = start_degree + packet * 11U;
        uint16_t distances[12];
        uint8_t raw[47];
        for (unsigned i = 0U; i < 12U; ++i) {
            unsigned angle = start + i;
            distances[i] = get_distance(angle % 360U, angle / 360U);
        }
        ld06_frame(raw, start * 100U, 1100U, distances);
        tick++;
        push(raw, 47U);
    }
}

static void test_ld06_inclusive_endpoint_and_error_recovery(void)
{
    reset();
    ld06_sweep(350U, 35U, front_object);
    n10p_obstacle_snapshot_t s = get_snapshot();
    assert(s.valid && s.count == 1U && s.front_known && s.blocked);
    assert(n10p_radar_valid_frames() == 35U && n10p_radar_bad_frames() == 0U);
    assert(s.obstacles[0].x_mm >= 109 && s.obstacles[0].x_mm <= 113);
    uint16_t distances[12] = {0};
    uint8_t raw[47];
    ld06_frame(raw, 1500U, 1100U, distances);
    raw[46] ^= 1U;
    push(raw, 47U);
    assert(!get_snapshot().valid && n10p_radar_bad_frames() == 1U);
    ld06_sweep(350U, 35U, front_object);
    assert(get_snapshot().valid);
    ld06_frame(raw, 1500U, 1100U, distances);
    raw[1] = 0U;
    push(raw, 47U);
    assert(!get_snapshot().valid && n10p_radar_bad_frames() == 2U);
    ld06_sweep(350U, 35U, front_object);
    assert(get_snapshot().valid);
}

static void read_captured_zero_status(uint8_t *raw)
{
    FILE *input = fopen("tests/artifacts/hardware-rx-09.bin", "rb");
    assert(input != NULL);
    assert(fseek(input, 1272L, SEEK_SET) == 0);
    assert(fread(raw, 1U, STL06_STATUS_FRAME_LEN, input) == STL06_STATUS_FRAME_LEN);
    assert(fclose(input) == 0);
    assert(raw[0] == 0x54U && raw[1] == 0x1FU);
    assert(raw[2] == 0U && raw[3] == 0U && raw[4] == 0U && raw[5] == 0x41U);
    assert(ld06_crc_bitwise(raw, STL06_STATUS_FRAME_LEN - 1U) == raw[5]);
}

static void test_stl06_zero_status_preserves_scan(void)
{
    uint8_t status[STL06_STATUS_FRAME_LEN];
    read_captured_zero_status(status);
    reset();
    ld06_sweep(350U, 35U, front_object);
    n10p_obstacle_snapshot_t before = get_snapshot();
    assert(before.valid);
    for (unsigned split = 1U; split < STL06_STATUS_FRAME_LEN; ++split) {
        push(status, split);
        push(status + split, STL06_STATUS_FRAME_LEN - split);
        n10p_obstacle_snapshot_t after = get_snapshot();
        assert(after.valid && after.scan_seq == before.scan_seq && after.count == before.count);
        assert(after.front_mm == before.front_mm && after.blocked == before.blocked);
        assert(memcmp(after.obstacles, before.obstacles, sizeof(before.obstacles)) == 0);
    }
    assert(n10p_radar_valid_frames() == 35U && n10p_radar_bad_frames() == 0U);
    ld06_sweep(15U, 33U, front_object);
    n10p_obstacle_snapshot_t after = get_snapshot();
    assert(after.valid && after.scan_seq == before.scan_seq + 1U);
    assert(n10p_radar_valid_frames() == 68U && n10p_radar_bad_frames() == 0U);

    tick += 300U;
    push(status, sizeof(status));
    assert(get_snapshot().valid);
    tick += 200U;
    push(status, sizeof(status));
    assert(!get_snapshot().valid && n10p_radar_bad_frames() == 0U);
    reset();
    for (unsigned report = 0U; report < 4U; ++report) {
        tick += 500U;
        push(status, sizeof(status));
    }
    after = get_snapshot();
    assert(!after.valid && after.age_ms == UINT32_MAX);
    assert(n10p_radar_valid_frames() == 0U && n10p_radar_bad_frames() == 0U);
}

static void test_stl06_bad_status_invalidates_scan(void)
{
    for (unsigned mutation = 0U; mutation < 5U; ++mutation) {
        uint8_t status[STL06_STATUS_FRAME_LEN];
        read_captured_zero_status(status);
        reset();
        ld06_sweep(350U, 35U, front_object);
        assert(get_snapshot().valid);
        if (mutation == 0U) status[5] ^= 1U;
        else if (mutation < 4U) {
            status[mutation + 1U] = 1U;
            status[5] = ld06_crc_bitwise(status, 5U);
        } else {
            status[1] = 0x1EU;
            status[5] = ld06_crc_bitwise(status, 5U);
        }
        push(status, sizeof(status));
        n10p_obstacle_snapshot_t after = get_snapshot();
        assert(!after.valid && after.count == 0U && n10p_radar_bad_frames() == 1U);
        ld06_sweep(350U, 35U, front_object);
        after = get_snapshot();
        assert(after.valid && after.scan_seq == 2U && n10p_radar_bad_frames() == 1U);
    }
}

static void test_hardware_ld06_capture(void)
{
    uint8_t capture[2048];
    FILE *input = fopen("tests/artifacts/hardware-rx.bin", "rb");
    assert(input != NULL);
    assert(fread(capture, 1U, sizeof(capture), input) == sizeof(capture));
    assert(fclose(input) == 0);
    reset();
    /* The captured ring's oldest complete frame is at 1929, timestamp 2915. */
    unsigned first = 1929U;
    const uint8_t noise[] = {0x14U, 0x15U, 0x00U, 0x54U, 0x54U};
    push(noise, sizeof(noise));
    for (unsigned turn = 0U; turn < 2U; ++turn) {
        for (unsigned packet = 0U; packet < 42U; ++packet) {
            uint8_t raw[47];
            for (unsigned byte = 0U; byte < 47U; ++byte) {
                raw[byte] = capture[(first + packet * 47U + byte) % sizeof(capture)];
            }
            assert(raw[0] == 0x54U && raw[1] == 0x2CU);
            assert(ld06_crc_bitwise(raw, 46U) == raw[46]);
            tick += 3U;
            push(raw, 47U);
        }
    }
    n10p_obstacle_snapshot_t s = get_snapshot();
    assert(n10p_radar_valid_frames() == 84U && n10p_radar_bad_frames() == 0U);
    assert(s.valid && s.scan_seq >= 1U && s.count > 0U);
    assert(s.front_known && s.blocked && s.front_mm < 500U);
    printf("captured LD06 replay: frames=%lu scans=%lu obstacles=%u front_mm=%lu\n",
           (unsigned long)n10p_radar_valid_frames(), (unsigned long)s.scan_seq,
           s.count, (unsigned long)s.front_mm);
}

int main(void)
{
    test_complete_scan_and_coordinates();
    test_crossing_packet_belongs_to_two_scans();
    test_seam_merge_before_sample_filter();
    test_raw_sample_centroid();
    test_nearest_eight_and_distance_split();
    test_front_uses_every_raw_sample();
    test_initialization_resets_partial_header();
    test_backward_angle_invalidates_scan();
    test_timeout_and_recovery();
    test_transport_failures_and_parser_reset();
    test_empty_scan_and_tick_wrap();
    test_invalid_endpoints_and_arbitrary_chunks();
    test_same_bin_radial_jump_is_not_an_object();
    test_fractional_angle_gap_splits_raw_groups();
    test_corrupt_header_invalidates_until_a_new_complete_scan();
    test_building_does_not_mutate_published_snapshot();
    test_ld06_inclusive_endpoint_and_error_recovery();
    test_stl06_zero_status_preserves_scan();
    test_stl06_bad_status_invalidates_scan();
    test_hardware_ld06_capture();
    printf("radar tests passed (%u snapshot checks)\n", checks);
    return 0;
}
