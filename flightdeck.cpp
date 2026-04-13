#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cstdarg>
#include <cmath>
#include <cctype>
#include <cerrno>
#include <memory>
#include <optional>
#include <thread>
#include <queue>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <string>
#include <algorithm>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "XPLMMenus.h"
#include "XPLMUtilities.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMPlugin.h"
#include "XPLMGraphics.h"
#include "XPLMDisplay.h"
#include "XPLMNavigation.h"
#include "XPImGui.h"
#include "imgui.h"

using std::string;

const float EPS = 1e-4;
const float PI = 3.14159265358979323846;
const double FEET_PER_M = 3.28084;

typedef uint8_t *bytes_t;

static uint32_t read_le_u32(const uint8_t *src) {
    uint32_t v;
    std::memcpy(&v, src, sizeof(v));
    return le32toh(v);
}

static void write_le_u32(uint8_t *dst, uint32_t value) {
    uint32_t v = htole32(value);
    std::memcpy(dst, &v, sizeof(v));
}

static void write_be_u16(uint8_t *dst, uint16_t value) {
    uint16_t v = htobe16(value);
    std::memcpy(dst, &v, sizeof(v));
}

static void write_be_u32(uint8_t *dst, uint32_t value) {
    uint32_t v = htobe32(value);
    std::memcpy(dst, &v, sizeof(v));
}

static void write_be_u64(uint8_t *dst, uint64_t value) {
    uint64_t v = htobe64(value);
    std::memcpy(dst, &v, sizeof(v));
}

double meter_to_feet(double m) {
    return m * FEET_PER_M;
}

double feet_to_meter(double ft) {
    return ft / FEET_PER_M;
}

double kts_to_mps(double v) {
    return v * 0.514444;
}

double mps_to_kts(double v) {
    return v / 0.514444;
}

void print_debug(const char *fmt, ...) {
    string prefixed = string("[Loupe Flightdeck] ") + fmt + "\n";
    const char *fmt_ = prefixed.c_str();
    va_list ap;
    va_start(ap, fmt);
    char *p = nullptr;
    const int rc = vasprintf(&p, fmt_, ap);
    va_end(ap);
    if (rc < 0 || !p) {
        XPLMDebugString("[Loupe Flightdeck] logging failed\n");
        return;
    }
    XPLMDebugString(p);
    free(p);
}

struct SocketAddr {
    uint32_t ip;
    uint16_t port;

    bool operator==(const SocketAddr &other) const {
        return ip == other.ip && port == other.port;
    }

    SocketAddr(struct sockaddr_in &addr): ip(addr.sin_addr.s_addr), port(addr.sin_port) {}

    struct sockaddr_in get_sockaddr_in() const {
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = ip;
        addr.sin_port = port;
        return addr;
    }
};

template<> struct std::hash<SocketAddr> {
    std::size_t operator()(const SocketAddr& k) const {
        return k.ip ^ (k.port << 1);
    }
};

typedef std::pair<SocketAddr, int32_t> subid_t;
template<> struct std::hash<subid_t> {
    std::size_t operator()(const subid_t& k) const {
        return std::hash<SocketAddr>{}(k.first) ^ k.second;
    }
};

struct Subscription {
    int32_t id;
    int32_t freq;
    string dataref_path;
    std::optional<uint8_t> idx;
    SocketAddr from;

    Subscription(int32_t id, int32_t freq,
                string dataref_path,
                std::optional<uint8_t> idx,
                SocketAddr from):
        id(id),
        freq(freq),
        dataref_path(std::move(dataref_path)),
        idx(idx),
        from(from) {}
};

std::queue<std::pair<float, subid_t>> stops;
std::unordered_map<subid_t, std::shared_ptr<Subscription>> id2sub; // id -> subscription
std::unordered_map<int32_t, std::unordered_map<subid_t, std::shared_ptr<Subscription>>> next_plan; // freq -> id -> subscription

static std::unordered_map<string, XPLMNavRef> navaid_table;
static bool navaid_table_initialized = false;

static void navaid_table_init() {
    if (navaid_table_initialized) return;
    navaid_table_initialized = true;
    navaid_table.clear();
    auto nav = XPLMGetFirstNavAid();
    char id[256];
    float lat, lng;
    while (nav != XPLM_NAV_NOT_FOUND) {
        XPLMGetNavAidInfo(nav, nullptr, &lat, &lng, nullptr, nullptr, nullptr, id, nullptr, nullptr);
        string id_str(id);
        if (navaid_table.count(id_str) == 0 ||
            (-162 < lng && lng < -68)) { // keep the US area for ambiguous navaids
            navaid_table[id_str] = nav;
        }
        nav = XPLMGetNextNavAid(nav);
    }
}

static void navaid_table_reset() {
    navaid_table.clear();
    navaid_table_initialized = false;
}

struct Event {
    virtual void handle() = 0;
    virtual ~Event() {}
};

struct CommandEvent: public Event {
    string cmd;
    
    explicit CommandEvent(string cmd): cmd(std::move(cmd)) {}
    ~CommandEvent() override {}

    void handle() override {
        XPLMCommandRef cmd_ref = XPLMFindCommand(cmd.c_str());
        if (!cmd_ref) {
            print_debug("command not found: %s", cmd.c_str());
            return;
        }
        XPLMCommandOnce(cmd_ref);
    }
};

struct DataSubscribeEvent: public Event {
    string dataref;
    int32_t freq;
    int32_t id;
    SocketAddr from;
    
    DataSubscribeEvent(string dataref, int32_t freq, int32_t id, SocketAddr from):
        dataref(std::move(dataref)), freq(freq), id(id), from(from) {}
    ~DataSubscribeEvent() override {}

    void handle() override {
        std::optional<uint8_t> idx;
        if (dataref.length() > 0 && dataref.back() == ']') {
            auto array_pos = dataref.find('[');
            if (array_pos != string::npos) {
                auto prefix = dataref.substr(0, array_pos);
                size_t n;
                auto idx_str = dataref.substr(array_pos + 1, dataref.length() - 2 - array_pos);
                int idx_num = -1;
                try {
                    idx_num = std::stoi(idx_str, &n);
                } catch (...) {
                    print_debug("invalid dataref index syntax: %s", dataref.c_str());
                    return;
                }
                if (n != idx_str.length() || idx_num < 0) {
                    return;
                }
                if (idx_num > 255) {
                    print_debug("dataref index out of range: %d (%s)", idx_num, dataref.c_str());
                    return;
                }
                dataref = prefix;
                idx = static_cast<uint8_t>(idx_num);
            }
        }
        auto dref = XPLMFindDataRef(dataref.c_str());
        if (!dref) {
            print_debug("dataref not found: %s", dataref.c_str());
            return;
        }
        auto sub = std::make_shared<Subscription>(id, freq, dataref, idx, from);
        next_plan[freq][std::make_pair(from, id)] = sub;
    }
};

double get_elevation() {
    auto dref = XPLMFindDataRef("sim/flightmodel/position/elevation");
    if (!dref) {
        return 0.0;
    }
    return XPLMGetDatad(dref);
}

float get_groundspeed() {
    auto dref = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
    if (!dref) {
        return 0.0f;
    }
    return XPLMGetDataf(dref);
}

struct TeleportEvent: public Event {
    string fix_id;
    float alt_ft;
    float gs_kt;

    TeleportEvent(string fix_id, float alt_ft, float gs_kt):
        fix_id(std::move(fix_id)),
        alt_ft(alt_ft),
        gs_kt(gs_kt) {}
    ~TeleportEvent() override {}

    void handle() override {
        XPLMNavType type;
        float lat, lng, elev;
        char name[256];
        navaid_table_init();
        const auto it = navaid_table.find(fix_id);
        if (it != navaid_table.end()) {
            XPLMGetNavAidInfo(it->second, &type, &lat, &lng, &elev, nullptr, nullptr, nullptr, name, nullptr);
            print_debug("teleporting to %s(type=%d, lat=%.6f, lng=%.6f, elev=%.2f)", name, type, lat, lng, elev);

            const double alt_m = feet_to_meter(alt_ft);
            const float gs = std::max(gs_kt, 80.0f);

            double x, y, z;
            XPLMWorldToLocal(lat, lng, alt_m, &x, &y, &z);
            auto local_x_dref = XPLMFindDataRef("sim/flightmodel/position/local_x");
            auto local_y_dref = XPLMFindDataRef("sim/flightmodel/position/local_y");
            auto local_z_dref = XPLMFindDataRef("sim/flightmodel/position/local_z");
            auto local_vx_dref = XPLMFindDataRef("sim/flightmodel/position/local_vx");
            auto local_vy_dref = XPLMFindDataRef("sim/flightmodel/position/local_vy");
            auto local_vz_dref = XPLMFindDataRef("sim/flightmodel/position/local_vz");
            auto psi_dref = XPLMFindDataRef("sim/flightmodel/position/psi");
            if (!local_x_dref || !local_y_dref || !local_z_dref ||
                !local_vx_dref || !local_vy_dref || !local_vz_dref ||
                !psi_dref) {
                print_debug("teleport failed: required dataref missing");
                return;
            }
            auto psi = XPLMGetDataf(psi_dref) * PI / 180;
            auto mps = kts_to_mps(gs);

            XPLMSetDatad(local_x_dref, x);
            XPLMSetDatad(local_y_dref, y);
            XPLMSetDatad(local_z_dref, z);

            XPLMSetDataf(local_vx_dref, mps * sin(psi));
            XPLMSetDataf(local_vz_dref, mps * -cos(psi));
            XPLMSetDataf(local_vy_dref, 0);
        } else {
            print_debug("invalid navaid/fix ID: %s", fix_id.c_str());
        }
    }
};

struct EventQueue {
    std::mutex lck;
    std::queue<std::unique_ptr<Event>> q;

    void push(std::unique_ptr<Event> event) {
        std::lock_guard<std::mutex> lk(lck);
        q.push(std::move(event));
    }

    bool pop(std::unique_ptr<Event> &event) {
        std::lock_guard<std::mutex> lk(lck);
        if (q.empty()) {
            return false;
        }
        event = std::move(q.front());
        q.pop();
        return true;
    }
};

struct UDPServer {
    int sockfd;
    std::atomic<bool> running;
    std::thread inbound_msg_loop;
    std::mutex socket_lock;

    virtual void on_message(bytes_t buffer, size_t len, SocketAddr from) = 0;

    UDPServer(int port): sockfd(-1), running(false), socket_lock() {
        if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            print_debug("socket creation failed");
            return;
        }

        int reuse = 1;
        setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        struct timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 200000; // 200ms recv timeout so stop() does not depend on socket shutdown behavior
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        struct sockaddr_in server_addr;

        memset(&server_addr, 0, sizeof(server_addr));

        server_addr.sin_family = AF_INET; // IPv4
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(port);

        if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)  {
            close(sockfd);
            sockfd = -1;
            print_debug("bind failed");
            return;
        }
    }

    bool start() {
        if (sockfd < 0) {
            fprintf(stderr, "[Loupe Flightdeck] UDP start skipped: socket is unavailable\n");
            return false;
        }
        if (running.exchange(true)) {
            return true;
        }

        const int fd = sockfd;

        inbound_msg_loop = std::thread([this, fd] {
            fprintf(stderr, "[Loupe Flightdeck] inbound message loop is started\n");
            const size_t BUFFER_SIZE = 65536;
            uint8_t buffer[BUFFER_SIZE];

            for (; running.load(std::memory_order_acquire);) {
                struct sockaddr_in client_addr{};
                socklen_t addr_len = sizeof(client_addr);
                int n = recvfrom(fd, buffer, BUFFER_SIZE, 0, (struct sockaddr *)&client_addr, &addr_len);
                if (n > 0) {
                    on_message(buffer, n, client_addr);
                } else {
                    if (!running.load(std::memory_order_acquire)) break;
                    if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
                    break;
                }
            }
            fprintf(stderr, "[Loupe Flightdeck] inbound message loop ended\n");
        });
        return true;
    }

    virtual ~UDPServer() = default;

    void stop() {
        int fd = -1;
        running.store(false, std::memory_order_release);
        {
            std::lock_guard<std::mutex> lk(socket_lock);
            fd = sockfd;
            sockfd = -1;
        }

        if (fd >= 0) {
            print_debug("shutting down UDP Server");
            shutdown(fd, SHUT_RDWR);
            close(fd);
        }
        if (inbound_msg_loop.joinable()) inbound_msg_loop.join();
    }

    void outbound(const std::vector<uint8_t> &msg, const struct sockaddr_in &addr) {
        int fd = -1;
        {
            std::lock_guard<std::mutex> lk(socket_lock);
            fd = sockfd;
        }
        if (fd < 0 || !running.load(std::memory_order_acquire)) {
            return;
        }
        sendto(fd, msg.data(), msg.size(), 0, (const struct sockaddr *)&addr, sizeof(addr));
    }
};

struct XPlaneS: public UDPServer {
    EventQueue events;

    static const int PORT = 40086;

    XPlaneS(): UDPServer(PORT) {}

    ~XPlaneS() override {
        stop();
    }

    void on_message(bytes_t buffer, size_t len, SocketAddr from) override {
        constexpr size_t HEADER_LEN = 5;
        if (len >= HEADER_LEN && std::memcmp(buffer, "RREF", 4) == 0 && buffer[4] == 0) {
            // RREF\0 + freq(4) + id(4) + dataref(400)
            if (len == HEADER_LEN + 8 + 400) {
                const bytes_t base = buffer + HEADER_LEN;
                const int32_t freq = static_cast<int32_t>(read_le_u32(base));
                const int32_t id = static_cast<int32_t>(read_le_u32(base + 4));
                const char *dataref_ptr = reinterpret_cast<const char *>(base + 8);
                const size_t max_len = 400;
                size_t dataref_len = 0;
                while (dataref_len < max_len && dataref_ptr[dataref_len] != '\0') {
                    dataref_len++;
                }
                if (dataref_len == 0) {
                    fprintf(stderr, "[Loupe Flightdeck] invalid RREF payload\n");
                    return;
                }
                events.push(std::make_unique<DataSubscribeEvent>(
                    string(dataref_ptr, dataref_len), freq, id, from));
            }
            return;
        }

        if (len > HEADER_LEN && std::memcmp(buffer, "CMND", 4) == 0 && buffer[4] == 0) {
            const bytes_t cmd_ptr = buffer + HEADER_LEN;
            const size_t max_len = len - HEADER_LEN;
            size_t cmd_len = 0;
            while (cmd_len < max_len && cmd_ptr[cmd_len] != '\0') {
                cmd_len++;
            }
            if (cmd_len == 0) {
                fprintf(stderr, "[Loupe Flightdeck] invalid CMND payload\n");
                return;
            }
            events.push(std::make_unique<CommandEvent>(
                string(reinterpret_cast<const char *>(cmd_ptr), cmd_len)));
            return;
        }

        fprintf(stderr, "[Loupe Flightdeck] invalid message header\n");
    }
};

struct GDL90: public UDPServer {
    std::optional<struct sockaddr_in> foreflight_addr;
    std::mutex foreflight_addr_lock;

    XPLMDataRef lat_dref, lng_dref,
                baro_alt_dref, geo_alt_dref,
                gs_dref, vs_dref, trk_dref,
                roll_dref, pitch_dref, hdg_dref,
                tas_dref, ias_dref;

    static const int PORT = 63093;

    GDL90(): UDPServer(PORT) {
        lat_dref = XPLMFindDataRef("sim/flightmodel/position/latitude");
        lng_dref = XPLMFindDataRef("sim/flightmodel/position/longitude");
        baro_alt_dref = XPLMFindDataRef("sim/flightmodel/misc/h_ind");
        geo_alt_dref = XPLMFindDataRef("sim/flightmodel/position/elevation");
        gs_dref = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
        vs_dref = XPLMFindDataRef("sim/flightmodel/position/vh_ind_fpm");
        trk_dref = XPLMFindDataRef("sim/flightmodel/position/hpath");
        roll_dref = XPLMFindDataRef("sim/flightmodel/position/phi");
        pitch_dref = XPLMFindDataRef("sim/flightmodel/position/theta");
        hdg_dref = XPLMFindDataRef("sim/flightmodel/position/mag_psi");
        ias_dref = XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed");
        tas_dref = XPLMFindDataRef("sim/flightmodel/position/true_airspeed");
    }

    ~GDL90() override {
        stop();
    }

    bool has_required_drefs() const {
        return lat_dref && lng_dref &&
               baro_alt_dref && geo_alt_dref &&
               gs_dref && vs_dref && trk_dref &&
               roll_dref && pitch_dref && hdg_dref &&
               tas_dref && ias_dref;
    }

    void on_message(bytes_t buffer, size_t len, SocketAddr from) override {
        auto f = from.get_sockaddr_in();
        string s((const char *)buffer, len);
        if (s.find("ForeFlight") != string::npos && s.find("GDL90") != string::npos) {
            auto pos = s.find("\"port\":");
            if (pos == string::npos) {
                fprintf(stderr, "[Loupe Flightdeck] malformed ForeFlight discovery payload (port missing)\n");
                return;
            }
            size_t value_start = pos + 7;
            while (value_start < s.size() && std::isspace(static_cast<unsigned char>(s[value_start]))) {
                value_start++;
            }
            size_t value_end = value_start;
            while (value_end < s.size() && std::isdigit(static_cast<unsigned char>(s[value_end]))) {
                value_end++;
            }
            if (value_start == value_end) {
                fprintf(stderr, "[Loupe Flightdeck] malformed ForeFlight discovery payload (port invalid)\n");
                return;
            }

            int port = 0;
            try {
                port = std::stoi(s.substr(value_start, value_end - value_start));
            } catch (...) {
                fprintf(stderr, "[Loupe Flightdeck] malformed ForeFlight discovery payload (port parse failed)\n");
                return;
            }
            if (port <= 0 || port > 65535) {
                fprintf(stderr, "[Loupe Flightdeck] malformed ForeFlight discovery payload (port out of range)\n");
                return;
            }
            f.sin_port = htons(port);
            {
                std::lock_guard<std::mutex> lk(foreflight_addr_lock);
                foreflight_addr = f;
            }
        }
    }

    static void set_24bit_signed(bytes_t base, uint32_t x) {
        base[0] = (x >> 16) & 0xff;
        base[1] = (x >> 8) & 0xff;
        base[2] = x & 0xff;
    }

    /// pasted from Garmin's GDL90 spec
    static uint16_t crc16_table[];
    static void crc_init() {
        static bool crc_initialized = false;
        if (crc_initialized) return;
        crc_initialized = true;
        uint16_t crc;
        for (uint16_t i = 0; i < 256; i++) {
            crc = (i << 8);
            for (uint16_t bitctr = 0; bitctr < 8; bitctr++) {
                crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0);
            }
            crc16_table[i] = crc;
        }
    }

    static uint16_t crc_compute(const bytes_t block, size_t len) {
        uint16_t crc = 0;
        for (size_t i = 0; i < len; i++) {
            crc = crc16_table[crc >> 8] ^ (crc << 8) ^ block[i];
        }
        return crc;
    }

    void send_msg(const bytes_t msg, size_t size, const struct sockaddr_in &addr) {
        std::vector<uint8_t> escaped{0x7e};

	    uint16_t crc = crc_compute(msg, size);
        std::vector<uint8_t> msg_with_crc(msg, msg + size);
        msg_with_crc.push_back(crc & 0xff);
        msg_with_crc.push_back((crc >> 8) & 0xff);

        for (auto b: msg_with_crc) {
            if (b == 0x7e || b == 0x7d) {
                b = b ^ 0x20;
                escaped.push_back(0x7d);
            }
            escaped.push_back(b);
        }
        escaped.push_back(0x7e);
        outbound(escaped, addr);
    }

    void send_heartbeat(const struct sockaddr_in &addr, bool pos_avail) {
        uint8_t buffer[7];
        buffer[0] = 0;
        buffer[1] = ((pos_avail ? 1 : 0) << 7) | 1;
        memset(buffer + 2, 0x0, 5); // all ignored by FF
        send_msg(buffer, sizeof(buffer), addr);
    }

    void send_idmsg(
            const struct sockaddr_in &addr,
            string name = "Loupe Flightdeck",
            uint64_t serial = 0x12345678, bool alt_msl = true) {
        uint8_t buffer[39];
        buffer[0] = 0x65;
        buffer[1] = 0;
        buffer[2] = 1;
        write_be_u64(buffer + 3, serial);
        name += string(16, ' ');
        memmove((char *)(buffer + 11), name.c_str(), 8);
        memmove((char *)(buffer + 19), name.c_str(), 16);
        write_be_u32(buffer + 35, alt_msl ? 1 : 0);
        send_msg(buffer, sizeof(buffer), addr);
    }

    void fill_traffic_report_payload(
            bytes_t buffer,
            float lat, float lng, float baro_alt,
            std::optional<float> gs, float vs, float trk,
            uint8_t misc = 0x9, uint32_t addr = 0x000000, string callsign = "N123AB",
            uint8_t nic_acc = 0xbb, uint8_t emitter = 0x1, uint8_t emergency = 0) {
        buffer[0] = 0x00; // no alert, ADS-B with self-assigned address

        set_24bit_signed(buffer + 1, addr); // 2/3/4

        auto lat_ = int32_t((lat < -90 ? -90 : (lat > 90 ? 90 : lat)) / 180 * 0x800000);
        if (lat_ < 0) lat_ = (0x1000000 + lat_) & 0xffffff;
        auto lng_ = int32_t((lng < -180 ? -180 : (lng > 180 ? 180 : lng)) / 180 * 0x800000);
        if (lng_ < 0) lng_ = (0x1000000 + lng_) & 0xffffff;
        set_24bit_signed(buffer + 4, lat_); // 5/6/7
        set_24bit_signed(buffer + 7, lng_); // 8/9/10
                                            //
        auto a = int32_t((baro_alt + 1000) / 25.0);
        a = a < 0 ? 0 : (a > 0xffe ? 0xffe : a);
        buffer[10] = (a >> 4) & 0xff;
        buffer[11] = ((a & 0xf) << 4) | (misc & 0xf);
        buffer[12] = nic_acc;
        auto hhh = int32_t(gs.has_value() ? \
                      (gs.value() < 0 ? 0 : (gs.value() > 0xffe ? 0xffe : gs.value())) : \
                      0xfff);
        int32_t vvv;
        if (vs < -32576) vvv = 0xe02;
        else if (vs > 32576) vvv = 0x1fe;
        else {
            vvv = int32_t(vs / 64);
            if (vvv < 0) vvv = (0x1000 + vvv) & 0xfff;
        }
        buffer[13] = (hhh >> 4) & 0xff;
        buffer[14] = ((hhh & 0xf) << 4) | ((vvv >> 8) & 0xf);
        buffer[15] = vvv & 0xff;
        buffer[16] = uint32_t(trk / 360 * 256) & 0xff;
        buffer[17] = emitter;
        callsign += string(8, ' ');
        memmove((char *)(buffer + 18), callsign.c_str(), 8); // tail number
        buffer[26] = (emergency & 0xf) << 4; // no emergency
    }

    void send_ownship_report(
            const struct sockaddr_in &addr,
            float lat, float lng, float baro_alt,
            std::optional<float> gs, float vs, float trk,
            uint8_t misc = 0x9, uint32_t icao_addr = 0x000000, string callsign = "N123AB",
            uint8_t nic_acc = 0xbb, uint8_t emitter = 0x1, uint8_t emergency = 0) {
        uint8_t buffer[28];
        buffer[0] = 10;
        fill_traffic_report_payload(buffer + 1, lat, lng, baro_alt,
                gs, vs, trk, misc, icao_addr, callsign, nic_acc, emitter, emergency);
        send_msg(buffer, sizeof(buffer), addr); // ownship
        //buffer[0] = 20;
        //send_msg(buffer, sizeof(buffer)); // also send as a traffic
    }

    void send_ownship_geo_alt(const struct sockaddr_in &addr, float alt) {
        uint8_t buffer[5];
        buffer[0] = 11;
        auto alt_ = int16_t(alt / 5);
        write_be_u16(buffer + 1, static_cast<uint16_t>(alt_));
        write_be_u16(buffer + 3, 0x0003);
        send_msg(buffer, sizeof(buffer), addr);
    }

    void send_ahrs_message(const struct sockaddr_in &addr, float roll, float pitch, float hdg, float ias, float tas, bool mag = true) {
        uint8_t buffer[12];
        buffer[0] = 0x65;
        buffer[1] = 0x1;
        auto roll_ = int32_t(roll * 10);
        roll_ = roll_ < -0x8000 ? -0x8000 : (roll_ > 0x7fff ? 0x7fff : roll_);
        write_be_u16(buffer + 2, static_cast<uint16_t>(int16_t(roll_)));

        auto pitch_ = int32_t(pitch * 10);
        pitch_ = pitch_ < -0x8000 ? -0x8000 : (pitch_ > 0x7fff ? 0x7fff : pitch_);
        write_be_u16(buffer + 4, static_cast<uint16_t>(int16_t(pitch_)));

        auto hdg_ = int32_t(hdg * 10);
        hdg_ = hdg_ < -3600 ? -3600 : (hdg_ > 3600 ? 3600 : hdg_);
        if (hdg_ < 0) hdg_ = (0x10000 + hdg_) & 0xffff;
        write_be_u16(buffer + 6, static_cast<uint16_t>(uint16_t(hdg_ & 0x7fff) | ((mag ? 1 : 0) << 15)));

        auto ias_ = uint16_t(ias < 0 ? 0 : (ias > 0x7fff ? 0x7fff : ias));
        write_be_u16(buffer + 8, ias_);

        auto tas_ = uint16_t(tas < 0 ? 0 : (tas > 0x7fff ? 0x7fff : tas));
        write_be_u16(buffer + 10, tas_);
        send_msg(buffer, sizeof(buffer), addr);
    }

    void update_1hz() {
        if (!has_required_drefs()) {
            return;
        }
        float lat = XPLMGetDatad(lat_dref);
        float lng = XPLMGetDatad(lng_dref);
        float baro_alt = XPLMGetDataf(baro_alt_dref); // Reported indicated altitude from sim.
        float geo_alt = meter_to_feet(XPLMGetDatad(geo_alt_dref));
        float gs = XPLMGetDataf(gs_dref) * 1.94384;
        float vs = XPLMGetDataf(vs_dref);
        float trk = XPLMGetDataf(trk_dref);
        std::optional<struct sockaddr_in> target;
        {
            std::lock_guard<std::mutex> lk(foreflight_addr_lock);
            target = foreflight_addr;
        }
        if (!target.has_value()) return;

        send_heartbeat(*target, true);
        send_ownship_report(*target, lat, lng, baro_alt, std::optional(gs), vs, trk);
        send_ownship_geo_alt(*target, geo_alt);
        send_idmsg(*target);
    }

    void update_ahrs() {
        if (!has_required_drefs()) {
            return;
        }
        float roll = XPLMGetDataf(roll_dref);
        float pitch = XPLMGetDataf(pitch_dref);
        float hdg = XPLMGetDataf(hdg_dref);
        float ias = XPLMGetDataf(ias_dref);
        float tas = XPLMGetDataf(tas_dref);
        std::optional<struct sockaddr_in> target;
        {
            std::lock_guard<std::mutex> lk(foreflight_addr_lock);
            target = foreflight_addr;
        }
        if (!target.has_value()) return;

        send_ahrs_message(*target, roll, pitch, hdg, ias, tas);
    }
};

uint16_t GDL90::crc16_table[256];

std::unique_ptr<XPlaneS> xps = nullptr;
std::unique_ptr<GDL90> gdl90 = nullptr;
XPLMFlightLoopID ctrl_loop_id = nullptr;
XPLMFlightLoopID data_loop_id = nullptr;
XPLMFlightLoopID gdl90_loop_id = nullptr;
XPLMCommandRef ir_training_cmd = nullptr;
XPLMCommandRef throttle_full_cmd = nullptr;
XPLMCommandRef prop_full_cmd = nullptr;
static bool plugin_enabled = false;
static bool command_handlers_registered = false;

static bool read_subscription_value(const Subscription &sub, float &value_out) {
    XPLMDataRef dref = XPLMFindDataRef(sub.dataref_path.c_str());
    if (!dref) {
        return false;
    }

    const XPLMDataTypeID dtype = XPLMGetDataRefTypes(dref);
    if (sub.idx.has_value()) {
        const int idx = sub.idx.value();
        if (dtype & xplmType_IntArray) {
            int value = 0;
            XPLMGetDatavi(dref, &value, idx, 1);
            value_out = static_cast<float>(value);
            return true;
        }
        if (dtype & xplmType_FloatArray) {
            XPLMGetDatavf(dref, &value_out, idx, 1);
            return true;
        }
        return false;
    }

    if (dtype & xplmType_Float) {
        value_out = XPLMGetDataf(dref);
        return true;
    }
    if (dtype & xplmType_Double) {
        value_out = static_cast<float>(XPLMGetDatad(dref));
        return true;
    }
    if (dtype & xplmType_Int) {
        value_out = static_cast<float>(XPLMGetDatai(dref));
        return true;
    }
    return false;
}

float ctrl_loop(float, float, int, void *) {
    if (!xps) {
        return 0;
    }
    for (;;) {
        std::unique_ptr<Event> event;
        if (!xps->events.pop(event)) {
            break;
        }
        event->handle();
    }
    return 0.01;
}

float data_loop(float, float, int, void *) {
    if (!xps) {
        return 0;
    }
    float elapsed = XPLMGetElapsedTime();
    std::unordered_map<SocketAddr, std::vector<std::pair<int32_t, std::shared_ptr<Subscription>>>> expired;
    while (!stops.empty() && elapsed + EPS > stops.front().first) {
        auto id = std::move(stops.front().second);
        stops.pop();
        const auto it = id2sub.find(id);
        if (it != id2sub.end()) {
            expired[id.first].push_back(std::make_pair(id.second, it->second));
        }
    }
    if (!expired.empty() && xps) {
        static const auto prefix = string("RREF,");

        for (const auto &[a, v]: expired) {
            std::vector<uint8_t> data_msg;
            data_msg.insert(data_msg.end(), prefix.begin(), prefix.end());
            data_msg.resize(prefix.length() + 8 * v.size());
            bytes_t ptr = &*data_msg.begin() + prefix.length();
            for (const auto &p: v) {
                write_le_u32(ptr, static_cast<uint32_t>(p.first));
                ptr += 4;
                float value = 0.0f;
                const auto &sub = p.second;
                if (sub) {
                    read_subscription_value(*sub, value);
                }
                uint32_t raw_v;
                std::memcpy(&raw_v, &value, sizeof(raw_v));
                uint32_t encoded = htole32(raw_v);
                std::memcpy(ptr, &encoded, sizeof(encoded));
                ptr += 4;
            }
            xps->outbound(data_msg, a.get_sockaddr_in());
        }
    }
    if (stops.empty()) {
        // load the schedule for the next second.
        id2sub.clear();
        std::vector<std::pair<float, subid_t>> all;
        for (const auto &[freq, m]: next_plan) {
            int f = std::max(std::min(freq, 100), 0);
            for (const auto &[id, sub]: m) {
                if (f) {
                    float inc = 1.0 / float(f) - EPS;
                    float t = 0;
                    for (int i = 0; i < f; i++, t += inc) {
                        all.push_back(std::make_pair(elapsed + t, id));
                    }
                    id2sub[id] = sub;
                }
            }
        }
        std::sort(all.begin(), all.end(), [](const auto &a, const auto &b) {
            return a.first < b.first;
        });
        for (auto &&p: all) {
            stops.push(p);
        }
    }
    if (stops.empty()) {
        return 1;
    }
    auto dt = stops.front().first - elapsed;
    return dt < EPS ? -1 : dt;
}

float gdl90_loop(float, float, int, void *) {
    if (!gdl90) {
        return 0;
    }
    static const int main_freq = 1;
    static const int ahrs_factor = 10;
    static const float interval = 1.0 / (main_freq * ahrs_factor);
    static int counter = 0;
    if (!counter++) {
        gdl90->update_1hz();
    }
    gdl90->update_ahrs();
    if (counter == ahrs_factor) counter = 0;
    return interval;
}

int set_throttle_full(XPLMCommandRef, XPLMCommandPhase phase, void *) {
    if (phase != xplm_CommandBegin) {
        return 1;
    }
    XPLMDataRef dref = XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio_all");
    if (dref) {
        XPLMSetDataf(dref, 1.0f);
    }
    return 0;
}

int set_prop_full(XPLMCommandRef, XPLMCommandPhase phase, void *) {
    if (phase != xplm_CommandBegin) {
        return 1;
    }
    XPLMDataRef dref = XPLMFindDataRef("sim/cockpit2/engine/actuators/prop_ratio_all");
    if (dref) {
        XPLMSetDataf(dref, 1.0f);
    }
    return 0;
}

static float foggle_orig_cloud_data[9];
static XPLMDataRef foggle_cloud_drefs[9];
static bool foggles_on = false;

static void foggles_reset() {
    if (foggles_on) {
        print_debug("resetting foggles (plugin disabled)...");
        for (int i = 0; i < 9; i++) {
            if (foggle_cloud_drefs[i]) XPLMSetDataf(foggle_cloud_drefs[i], foggle_orig_cloud_data[i]);
        }
        XPLMCommandRef cmd_ref = XPLMFindCommand("sim/GPS/g1000n1_popup");
        if(cmd_ref) XPLMCommandOnce(cmd_ref);
        cmd_ref = XPLMFindCommand("sim/GPS/g1000n3_popup");
        if(cmd_ref) XPLMCommandOnce(cmd_ref);
        foggles_on = false;
    }
}

int toggle_ir_training(XPLMCommandRef, XPLMCommandPhase phase, void *) {
    if (phase != xplm_CommandBegin) {
        return 1;
    }
    XPLMCommandRef cmd_ref = XPLMFindCommand("sim/GPS/g1000n1_popup");
    if (cmd_ref) XPLMCommandOnce(cmd_ref);
    cmd_ref = XPLMFindCommand("sim/GPS/g1000n3_popup");
    if (cmd_ref) XPLMCommandOnce(cmd_ref);

    static const float cloud_ir[9] = {0.0, 10000.0, 11000.0, 10000.0, 11000.0, 21000.0, 6, 6, 6};

    // Initialize drefs if needed (idempotent)
    for (int i = 0; i < 3; i++) {
        char buff[64];
        sprintf(buff, "sim/weather/cloud_base_msl_m[%d]", i);
        foggle_cloud_drefs[i] = XPLMFindDataRef(buff);
        sprintf(buff, "sim/weather/cloud_tops_msl_m[%d]", i);
        foggle_cloud_drefs[i + 3] = XPLMFindDataRef(buff);
        sprintf(buff, "sim/weather/cloud_coverage[%d]", i);
        foggle_cloud_drefs[i + 6] = XPLMFindDataRef(buff);
    }

    if (foggles_on) {
        print_debug("foggles off...");
        for (int i = 0; i < 9; i++) {
            if (foggle_cloud_drefs[i]) {
                XPLMSetDataf(foggle_cloud_drefs[i], foggle_orig_cloud_data[i]);
            }
        }
    } else {
        auto sim_speed_dref = XPLMFindDataRef("sim/time/sim_speed");
        bool paused = sim_speed_dref ? (XPLMGetDatai(sim_speed_dref) == 0) : false;
        print_debug("foggles on...");
        for (int i = 0; i < 9; i++) {
            if (!foggle_cloud_drefs[i]) {
                continue;
            }
            if (!paused) {
                foggle_orig_cloud_data[i] = XPLMGetDataf(foggle_cloud_drefs[i]);
            }
            XPLMSetDataf(foggle_cloud_drefs[i], cloud_ir[i]);
        }
    }
    foggles_on ^= 1;
    return 0;
}

XPLMWindowID teleport_window = 0;
char fix_buffer[32] = "";
float alt_val = 0.0f;
float gs_val = 0.0f;

int teleport_window_on_mouse_click(XPLMWindowID id, int x, int y, XPLMMouseStatus status, void *refcon) {
    if (!plugin_enabled) return 0;
    if (status == xplm_MouseDown) {
        XPLMTakeKeyboardFocus(id);
    }
    return XPImGui::HandleMouseClick(id, x, y, status, refcon);
}

int teleport_window_on_right_click(XPLMWindowID, int, int, XPLMMouseStatus, void *) {
    if (!plugin_enabled) return 0;
    return 0;
}

int teleport_window_on_mouse_wheel(XPLMWindowID, int, int, int, int, void *) {
    if (!plugin_enabled) return 0;
    return 0;
}

XPLMCursorStatus teleport_window_on_cursor(XPLMWindowID, int, int, void *) {
    return xplm_CursorDefault;
}

void teleport_window_on_keyboard(XPLMWindowID id, char key, XPLMKeyFlags flag, char virtualKey, void *refcon, int losing_focus) {
    if (!plugin_enabled) return;
    XPImGui::HandleKey(id, key, flag, virtualKey, refcon, losing_focus);
}

void teleport_window_on_draw(XPLMWindowID id, void* refcon) {
    if (!plugin_enabled) return;
    if (!XPImGui::NewFrame(id)) return;

    // Create a fullscreen window inside the X-Plane window
    // Flags: NoTitleBar | NoResize | NoMove (since the container X-Plane window handles this)
    ImGui::Begin("Teleport Content", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBackground);
    
    ImGui::Text("Enter the fix:");
    ImGui::InputText("##fix", fix_buffer, sizeof(fix_buffer));
    
    ImGui::Text("Altitude (ft.):");
    ImGui::InputFloat("##alt", &alt_val, 100.0f, 1000.0f, "%.0f");
    ImGui::SameLine();
    if (ImGui::Button("Current##alt")) {
        alt_val = meter_to_feet(get_elevation());
    }
    
    ImGui::Text("GS (kts.):");
    ImGui::InputFloat("##gs", &gs_val, 10.0f, 50.0f, "%.0f");
    ImGui::SameLine();
    if (ImGui::Button("Current##gs")) {
        gs_val = mps_to_kts(std::max(0.0f, get_groundspeed()));
    }
    
    ImGui::Separator();
    ImGui::Spacing();
    
    if (ImGui::Button("Go!", ImVec2(120, 35))) {
        string id_str(fix_buffer);
        if (xps) {
            xps->events.push(std::make_unique<TeleportEvent>(id_str, alt_val, gs_val));
        }
    }

    ImGui::End();
    XPImGui::Render();
}

void menu_handler(void *, void *inItemRef) {
    if (!plugin_enabled) return;
    auto id = uintptr_t(inItemRef);
    switch (id) {
        case 0: {
            XPLMCommandRef cmd_ref = XPLMFindCommand("lfd/toggle_imc_foggles");
            if (cmd_ref) XPLMCommandOnce(cmd_ref);
        }
        break;
        case 1: {
            if (!xps) return; // Plugin disabled check
            if (!teleport_window) {
                XPImGui::Init();
                int x0 = 500, y0 = 500;
                int w = 300, h = 250; // Slightly taller for ImGui spacing
                XPLMCreateWindow_t win{};
                win.structSize = sizeof(win);
                win.left = x0;
                win.bottom = y0 - h;
                win.right = x0 + w;
                win.top = y0;
                win.visible = 1;
                win.drawWindowFunc = teleport_window_on_draw;
                win.handleMouseClickFunc = teleport_window_on_mouse_click;
                win.handleRightClickFunc = teleport_window_on_right_click;
                win.handleMouseWheelFunc = teleport_window_on_mouse_wheel;
                win.handleKeyFunc = teleport_window_on_keyboard;
                win.handleCursorFunc = teleport_window_on_cursor;
                win.refcon = nullptr;
                win.layer = xplm_WindowLayerFloatingWindows;
                teleport_window = XPLMCreateWindowEx(&win);
                if (!teleport_window) {
                    print_debug("failed to create teleport window");
                    XPImGui::Shutdown();
                    return;
                }
                XPLMSetWindowPositioningMode(teleport_window, xplm_WindowPositionFree, -1);
                XPLMSetWindowResizingLimits(teleport_window, w, h, w, h);
                XPLMSetWindowTitle(teleport_window, "Teleport To");
                XPLMTakeKeyboardFocus(teleport_window);
                
                // Initialize defaults
                if (strlen(fix_buffer) == 0) strcpy(fix_buffer, "KSEA");
                alt_val = meter_to_feet(get_elevation());
                gs_val = mps_to_kts(std::max(0.0f, get_groundspeed()));
            } else {
                XPLMSetWindowIsVisible(teleport_window, 1);
                XPLMTakeKeyboardFocus(teleport_window);
            }
        }
        break;
    }
}

XPLMMenuID menu_id = nullptr;
int menu_idx = -1;

static void unregister_command_handlers() {
    if (!command_handlers_registered) {
        return;
    }
    if (ir_training_cmd) XPLMUnregisterCommandHandler(ir_training_cmd, toggle_ir_training, false, nullptr);
    if (throttle_full_cmd) XPLMUnregisterCommandHandler(throttle_full_cmd, set_throttle_full, false, nullptr);
    if (prop_full_cmd) XPLMUnregisterCommandHandler(prop_full_cmd, set_prop_full, false, nullptr);
    command_handlers_registered = false;
}

static void destroy_flight_loops() {
    if (ctrl_loop_id) {
        XPLMDestroyFlightLoop(ctrl_loop_id);
        ctrl_loop_id = nullptr;
    }
    if (data_loop_id) {
        XPLMDestroyFlightLoop(data_loop_id);
        data_loop_id = nullptr;
    }
    if (gdl90_loop_id) {
        XPLMDestroyFlightLoop(gdl90_loop_id);
        gdl90_loop_id = nullptr;
    }
}

static void destroy_teleport_window() {
    if (teleport_window) {
        XPLMSetWindowIsVisible(teleport_window, 0);
        XPLMDestroyWindow(teleport_window);
        teleport_window = 0;
    }
    XPImGui::Shutdown();
}

static void stop_udp_servers() {
    if (xps) {
        xps.reset();
        print_debug("XPS is disabled");
    }
    if (gdl90) {
        gdl90.reset();
        print_debug("GDL90 is disabled");
    }
}

static void clear_runtime_state() {
    std::queue<std::pair<float, subid_t>> empty_stops;
    std::swap(stops, empty_stops);
    id2sub.clear();
    next_plan.clear();
    navaid_table_reset();
}

static void cleanup_plugin_runtime() {
    plugin_enabled = false;
    unregister_command_handlers();
    foggles_reset();
    destroy_flight_loops();
    destroy_teleport_window();
    stop_udp_servers();
    clear_runtime_state();
}

PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc) {
    strcpy(outName, "Loupe Flightdeck");
    strcpy(outSig, "com.determinant.loupeflightdeck");
    strcpy(outDesc, "A plugin that reimplements X-Plane's basic UDP API and GDL90 support.");

    menu_idx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "Loupe Flightdeck", 0, 1);
    if (menu_idx < 0) {
        print_debug("failed to append plugin menu item");
        return 0;
    }
    menu_id = XPLMCreateMenu(
            "Loupe Flightdeck",
            XPLMFindPluginsMenu(),
            menu_idx,
            menu_handler,
            0);
    if (!menu_id) {
        print_debug("failed to create plugin menu");
        XPLMRemoveMenuItem(XPLMFindPluginsMenu(), menu_idx);
        menu_idx = -1;
        return 0;
    }

    XPLMAppendMenuItem(menu_id, "Toggle Foggles", (void *)0, 1);
    XPLMAppendMenuItem(menu_id, "Teleport to Fix", (void *)1, 1);
    
    ir_training_cmd = XPLMCreateCommand(
            "lfd/toggle_imc_foggles",
            "Toggle IR training mode. This will toggle the outside vision (IMC and back to the original condition) and also toggle the G1000 PFD/MFD display.");
    throttle_full_cmd = XPLMCreateCommand(
            "lfd/throttle_full",
            "Set throttle to full (for all engines).");
    prop_full_cmd = XPLMCreateCommand(
            "lfd/prop_full",
            "Set prop to full (for all engines).");

    return 1;
}

PLUGIN_API void	XPluginStop() {
    cleanup_plugin_runtime();
    if (menu_id) {
        XPLMDestroyMenu(menu_id);
        menu_id = nullptr;
    }
    if (menu_idx >= 0) {
        XPLMRemoveMenuItem(XPLMFindPluginsMenu(), menu_idx);
        menu_idx = -1;
    }
}

PLUGIN_API int XPluginEnable() {
    if (plugin_enabled) return 1;
    // Enabling may be retried after a partial failure. Always start from a clean runtime state.
    cleanup_plugin_runtime();
    GDL90::crc_init();

    xps = std::make_unique<XPlaneS>();
    if (!xps->start()) {
        print_debug("failed to start X-Plane UDP server");
        cleanup_plugin_runtime();
        return 0;
    }
    gdl90 = std::make_unique<GDL90>();
    if (!gdl90->start()) {
        print_debug("failed to start GDL90 UDP server");
        cleanup_plugin_runtime();
        return 0;
    }

    XPLMCreateFlightLoop_t loop_cfg;
    loop_cfg.structSize = sizeof(XPLMCreateFlightLoop_t);
    loop_cfg.phase = xplm_FlightLoop_Phase_AfterFlightModel;
    loop_cfg.refcon = nullptr;
    loop_cfg.callbackFunc = ctrl_loop;
    ctrl_loop_id = XPLMCreateFlightLoop(&loop_cfg);
    if (!ctrl_loop_id) {
        print_debug("failed to create ctrl flight loop");
        cleanup_plugin_runtime();
        return 0;
    }
    XPLMScheduleFlightLoop(ctrl_loop_id, -1, 1);

    loop_cfg.callbackFunc = data_loop;
    data_loop_id = XPLMCreateFlightLoop(&loop_cfg);
    if (!data_loop_id) {
        print_debug("failed to create data flight loop");
        cleanup_plugin_runtime();
        return 0;
    }
    XPLMScheduleFlightLoop(data_loop_id, -1, 1);

    loop_cfg.callbackFunc = gdl90_loop;
    gdl90_loop_id = XPLMCreateFlightLoop(&loop_cfg);
    if (!gdl90_loop_id) {
        print_debug("failed to create GDL90 flight loop");
        cleanup_plugin_runtime();
        return 0;
    }
    XPLMScheduleFlightLoop(gdl90_loop_id, -1, 1);

    if (ir_training_cmd) XPLMRegisterCommandHandler(ir_training_cmd, toggle_ir_training, false, nullptr);
    if (throttle_full_cmd) XPLMRegisterCommandHandler(throttle_full_cmd, set_throttle_full, false, nullptr);
    if (prop_full_cmd) XPLMRegisterCommandHandler(prop_full_cmd, set_prop_full, false, nullptr);
    command_handlers_registered = true;

    plugin_enabled = true;
    return 1;
}

PLUGIN_API void XPluginDisable() {
    cleanup_plugin_runtime();
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int msg, void*) {}
