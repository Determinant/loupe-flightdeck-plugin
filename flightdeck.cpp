#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cstdarg>
#include <cmath>
#include <memory>
#include <optional>
#include <thread>
#include <condition_variable>
#include <queue>
#include <mutex>
#include <unordered_map>
#include <string>
#include <algorithm>
#include <regex>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#if IBM
    #include <GL/wglew.h>
    #include <Windows.h>
#endif
#if LIN
    #include <GL/gl.h>
#elif __GNUC__
    #include <OpenGL/gl.h>
#else
    #include <gl/GL.h>
#endif

#include "XPLMMenus.h"
#include "XPLMUtilities.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMPlugin.h"
#include "XPLMGraphics.h"
#include "XPLMDisplay.h"
#include "XPLMNavigation.h"

using std::string;

const float EPS = 1e-4;
const float PI = 3.14159265358979323846;
const double FEET_PER_M = 3.28084;

typedef uint8_t *bytes_t;

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

string to_string_with_precision(const double v, const int n = 4) {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << v;
    return std::move(out).str();
}

void print_debug(const char *fmt, ...) {
    string prefixed = string("[Loupe Flightdeck] ") + fmt + "\n";
    const char *fmt_ = prefixed.c_str();
    va_list ap;
    va_start(ap, fmt);
    char *p;
    vasprintf(&p, fmt_, ap);
    va_end(ap);
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
        struct sockaddr_in addr;
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
    XPLMDataRef dataref;
    XPLMDataTypeID datatype;
    std::optional<uint8_t> idx;
    SocketAddr from;

    Subscription(int32_t id, int32_t freq,
                XPLMDataRef dataref,
                XPLMDataTypeID datatype,
                std::optional<uint8_t> idx,
                SocketAddr from): id(id), freq(freq),
                                dataref(dataref), datatype(datatype), idx(idx), from(from) {}

    void print() {
        string output = "<sub ";
        output += "id=" + std::to_string(id) + \
                ",freq=" + std::to_string(freq) + \
                ",dataref=" + std::to_string((uintptr_t)dataref) + \
                ",datatype=" + std::to_string((uintptr_t)datatype) + \
                (idx.has_value() ? ",idx=" + std::to_string(idx.value()) : "")+ ">";
        print_debug(output.c_str());
    }
};

std::queue<std::pair<float, subid_t>> stops;
std::unordered_map<subid_t, std::shared_ptr<Subscription>> id2sub; // id -> subscription
std::unordered_map<int32_t, std::unordered_map<subid_t, std::shared_ptr<Subscription>>> next_plan; // freq -> id -> subscription

struct Event {
    virtual void handle() = 0;
    virtual ~Event() {}
};

struct CommandEvent: public Event {
    string cmd;
    
    CommandEvent(const char *cmd): cmd(cmd) {}
    ~CommandEvent() override {}

    void handle() override {
        XPLMCommandRef cmd_ref = XPLMFindCommand(cmd.c_str());
        if (!cmd_ref) {
            print_debug("command not found: %s", cmd.c_str());
        }
        //print_debug("executing command: %s %x", cmd.c_str(), cmd_ref);
        XPLMCommandOnce(cmd_ref);
    }
};

struct DataSubscribeEvent: public Event {
    string dataref;
    int32_t freq;
    int32_t id;
    SocketAddr from;
    
    DataSubscribeEvent(const char *dataref, int32_t freq, int32_t id, SocketAddr from):
        dataref(dataref), freq(freq), id(id), from(from) {}
    ~DataSubscribeEvent() override {}

    void handle() override {
        std::optional<uint8_t> idx;
        if (dataref.length() > 0 && dataref.back() == ']') {
            auto array_pos = dataref.find('[');
            if (array_pos != string::npos) {
                auto prefix = dataref.substr(0, array_pos);
                size_t n;
                auto idx_str = dataref.substr(array_pos + 1, dataref.length() - 2 - array_pos);
                auto idx_num = std::stoi(idx_str, &n);
                if (n != idx_str.length() || idx_num < 0) {
                    return;
                }
                dataref = prefix;
                idx = std::optional(idx_num);
            }
        }
        auto dref = XPLMFindDataRef(dataref.c_str());
        auto dtype = XPLMGetDataRefTypes(dref);
        if (dref) {
            std::shared_ptr<Subscription> sub{
                new Subscription(id, freq, dref, dtype, idx, from)};
            //sub->print();
            next_plan[freq][std::make_pair(from, id)] = sub;
        } else {
            print_debug("dataref not found: %s", dataref.c_str());
        }
    }
};

std::unordered_map<string, XPLMNavRef> navaid_table;

void navaid_table_init() {
    static bool navaid_table_initialized = false;
    if (navaid_table_initialized) return;
    navaid_table_initialized = true;
    auto nav = XPLMGetFirstNavAid();
    char id[32];
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

double get_elevation() {
    auto dref = XPLMFindDataRef("sim/flightmodel/position/elevation");
    return XPLMGetDatad(dref);
}

float get_groundspeed() {
    auto dref = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
    return XPLMGetDataf(dref);
}

struct TeleportEvent: public Event {
    string id;
    string alt;
    string gs;

    TeleportEvent(string id, string alt, string gs): id(id), alt(alt), gs(gs) {};
    ~TeleportEvent() override {}

    void handle() override {
        XPLMNavType type;
        float lat, lng, elev;
        char name[256];
        navaid_table_init();
        auto it = navaid_table.find(id);
        if (it != navaid_table.end()) {
            XPLMGetNavAidInfo(it->second, &type, &lat, &lng, &elev, nullptr, nullptr, nullptr, name, nullptr);
            print_debug("teleporting to %s(type=%d, lat=%.6f, lng=%.6f, elev=%.2f)", name, type, lat, lng, elev);
            auto sim_speed_dref = XPLMFindDataRef("sim/time/sim_speed");
            XPLMSetDatai(sim_speed_dref, 0);

            size_t n;

            auto alt = std::stod(this->alt, &n);
            if (n == this->alt.length()) {
                alt = feet_to_meter(alt);
            } else {
                alt = get_elevation();
            }

            auto gs = std::stof(this->gs, &n);
            if (n != this->gs.length()) {
                gs = mps_to_kts(get_groundspeed());
            }
            if (gs < 80) {
                gs = 80;
            }

            double x, y, z;
            XPLMWorldToLocal(lat, lng, alt, &x, &y, &z);
            auto local_x_dref = XPLMFindDataRef("sim/flightmodel/position/local_x");
            auto local_y_dref = XPLMFindDataRef("sim/flightmodel/position/local_y");
            auto local_z_dref = XPLMFindDataRef("sim/flightmodel/position/local_z");
            auto local_vx_dref = XPLMFindDataRef("sim/flightmodel/position/local_vx");
            auto local_vy_dref = XPLMFindDataRef("sim/flightmodel/position/local_vy");
            auto local_vz_dref = XPLMFindDataRef("sim/flightmodel/position/local_vz");
            auto psi_dref = XPLMFindDataRef("sim/flightmodel/position/psi");
            auto psi = XPLMGetDataf(psi_dref) * PI / 180;
            auto mps = kts_to_mps(gs);

            XPLMSetDatad(local_x_dref, x);
            XPLMSetDatad(local_y_dref, y);
            XPLMSetDatad(local_z_dref, z);

            XPLMSetDataf(local_vx_dref, mps * sin(psi));
            XPLMSetDataf(local_vz_dref, mps * -cos(psi));
            XPLMSetDataf(local_vy_dref, 0);
        } else {
            print_debug("invalid navaid/fix ID: %s", id.c_str());
        }
    }
};

struct EventQueue {
    std::mutex lck;
    std::queue<std::unique_ptr<Event>> q;

    void push(std::unique_ptr<Event> event) {
        lck.lock();
        q.push(std::move(event));
        lck.unlock();
    }

    std::optional<std::unique_ptr<Event>> pop() {
        lck.lock();
        if (q.empty()) {
            lck.unlock();
            return std::optional<std::unique_ptr<Event>>{};
        }
        std::unique_ptr<Event> event = std::move(q.front());
        q.pop();
        lck.unlock();
        return event;
    }
};

struct UDPServer {
    int sockfd;
    std::thread inbound_msg_loop;
    std::thread outbound_msg_loop;

    std::queue<std::pair<std::vector<uint8_t>, struct sockaddr_in>> outbound_msgs;
    std::condition_variable outbound_cv;
    std::mutex outbound_lock;
    bool outbound_readable;
    bool outbound_exit;
    
    static const size_t MAX_OUTBOUND_QUEUED = 1024;

    virtual void on_message(bytes_t buffer, size_t len, SocketAddr from) = 0;

    UDPServer(int port): outbound_msgs(), outbound_cv(), outbound_lock(),
                outbound_readable(false), outbound_exit(false) {
        if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            print_debug("socket creation failed");
            return;
        }

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

        inbound_msg_loop = std::thread([this] {
            print_debug("inbound message loop is started");
            const size_t BUFFER_SIZE = 65536;
            socklen_t addr_len = sizeof(sockaddr_in);
            struct sockaddr_in client_addr;
            uint8_t buffer[BUFFER_SIZE];

            for (;;) {
                int n = recvfrom(sockfd, buffer, BUFFER_SIZE, 0, (struct sockaddr *)&client_addr, &addr_len);
                if (n > 0) {
                    on_message(buffer, n, client_addr);
                } else {
                    break;
                }
            }
            print_debug("inbound message loop ended");
        });

        outbound_msg_loop = std::thread([this] {
            print_debug("outbound message loop is started");
            const socklen_t addr_len = sizeof(sockaddr_in);
            for (;;) {
                std::unique_lock lk{outbound_lock};
                outbound_cv.wait(lk, [this] { return outbound_readable || outbound_exit; });
                if (outbound_exit) {
                    lk.unlock();
                    break;
                }
                while (!outbound_msgs.empty()) {
                    auto p = std::move(outbound_msgs.front());
                    const auto &data_msg = p.first;
                    struct sockaddr_in &client_addr = p.second;
                    outbound_msgs.pop();
                    sendto(sockfd, &*data_msg.begin(), data_msg.size(),
                        MSG_CONFIRM, (const struct sockaddr *)&client_addr, addr_len);
                }
                outbound_readable = false;
                lk.unlock();
            }
            print_debug("outbound message loop ended");
        });
    }

    virtual ~UDPServer() {
        if (sockfd >= 0) {
            print_debug("shutting down UDP Server");
            shutdown(sockfd, SHUT_RDWR);
            close(sockfd);
            inbound_msg_loop.join();

            outbound_lock.lock();
            outbound_exit = true;
            outbound_cv.notify_all();
            outbound_lock.unlock();
            outbound_msg_loop.join();
        }
    }

    void outbound(std::vector<uint8_t> &&msg, struct sockaddr_in &&addr) {
        outbound_lock.lock();
        if (outbound_msgs.size() < MAX_OUTBOUND_QUEUED) {
            outbound_msgs.push({msg, addr});
            outbound_readable = true;
            outbound_cv.notify_all();
        } else {
            print_debug("dropping outbound UDP msg");
        }
        outbound_lock.unlock();
    }
};

struct XPlaneS: public UDPServer {
    EventQueue events;

    static const int PORT = 40086;

    XPlaneS(): UDPServer(PORT) {}

    void on_message(bytes_t buffer, size_t len, SocketAddr from) override {
        //print_debug("got a message %lu", len);
        size_t prefix = 0;
        for (bytes_t ptr = buffer; ptr < buffer + len; ptr++) {
            if (*ptr == 0) {
                prefix = ptr - buffer;
                break;
            }
        }
        if (prefix == 4) {
            const char *str = (const char *)buffer;
            if (!strcmp(str, "RREF")) {
                //print_debug("got a ref registration");
                if (len == 4 + 1 + 4 * 2 + 400) {
                    bytes_t base = buffer + prefix + 1;
                    int32_t freq = le32toh(*(uint32_t *)base);
                    int32_t id = le32toh(*(uint32_t *)(base + 4));
                    buffer[len - 1] = 0;
                    events.push(std::unique_ptr<Event>(
                        new DataSubscribeEvent((const char *)base + 8, freq, id, from)));
                }
                return;
            }
            if (!strcmp(str, "CMND")) {
                //print_debug("got a command");
                buffer[len - 1] = 0;
                events.push(std::unique_ptr<Event>(
                    new CommandEvent((const char *)buffer + prefix + 1)));
                return;
            }
        }
        print_debug("invalid message header");
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

    void on_message(bytes_t buffer, size_t len, SocketAddr from) override {
        auto f = from.get_sockaddr_in();
        string s = (const char *)buffer;
        if (s.find("ForeFlight") != string::npos && s.find("GDL90") != string::npos) {
            auto pos = s.find("\"port\":");
            auto port = std::stoi(s.substr(pos + 7));
            f.sin_port = htons(port);
            //inet_pton(AF_INET, "127.0.0.1", &f.sin_addr);
            foreflight_addr_lock.lock();
            foreflight_addr = f;
            foreflight_addr_lock.unlock();
            char addr[128];
            inet_ntop(AF_INET, &f.sin_addr, addr, 128);
            //print_debug("found foreflight at %s:%d", addr, ntohs(f.sin_port));
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

    void send_msg(const bytes_t msg, size_t size) {
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
        struct sockaddr_in addr = foreflight_addr.value();
        outbound(std::move(escaped), std::move(addr));
    }

    void send_heartbeat(bool pos_avail) {
        uint8_t buffer[7];
        buffer[0] = 0;
        buffer[1] = ((pos_avail ? 1 : 0) << 7) | 1;
        memset(buffer + 2, 0x0, 5); // all ignored by FF
        send_msg(buffer, sizeof(buffer));
    }

    void send_idmsg(
            string name = "Loupe Flightdeck",
            uint64_t serial = 0x12345678, bool alt_msl = true) {
        uint8_t buffer[39];
        buffer[0] = 0x65;
        buffer[1] = 0;
        buffer[2] = 1;
        *((uint64_t *)(buffer + 3)) = htobe64(serial);
        name += string(16, ' ');
        memmove((char *)(buffer + 11), name.c_str(), 8);
        memmove((char *)(buffer + 19), name.c_str(), 16);
        *((uint32_t *)(buffer + 35)) = htobe32(alt_msl ? 1 : 0);
        send_msg(buffer, sizeof(buffer));
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
            float lat, float lng, float baro_alt,
            std::optional<float> gs, float vs, float trk,
            uint8_t misc = 0x9, uint32_t addr = 0x000000, string callsign = "N123AB",
            uint8_t nic_acc = 0xbb, uint8_t emitter = 0x1, uint8_t emergency = 0) {
        uint8_t buffer[28];
        buffer[0] = 10;
        fill_traffic_report_payload(buffer + 1, lat, lng, baro_alt,
                gs, vs, trk, misc, addr,  callsign, nic_acc, emitter, emergency);
        send_msg(buffer, sizeof(buffer)); // ownship
        //buffer[0] = 20;
        //send_msg(buffer, sizeof(buffer)); // also send as a traffic
    }

    void send_ownship_geo_alt(float alt) {
        uint8_t buffer[5];
        buffer[0] = 11;
        auto alt_ = int16_t(alt / 5);
        *(uint16_t *)(buffer + 1) = htobe16(alt_);
        *(uint16_t *)(buffer + 3) = htobe16(0x0003);
        send_msg(buffer, sizeof(buffer));
    }

    void send_ahrs_message(float roll, float pitch, float hdg, float ias, float tas, bool mag = true) {
        uint8_t buffer[12];
        buffer[0] = 0x65;
        buffer[1] = 0x1;
        auto roll_ = int32_t(roll * 10);
        roll_ = roll_ < -0x8000 ? -0x8000 : (roll_ > 0x7fff ? 0x7fff : roll_);
        *(int16_t *)(buffer + 2) = htobe16(int16_t(roll_));

        auto pitch_ = int32_t(pitch * 10);
        pitch_ = pitch_ < -0x8000 ? -0x8000 : (pitch_ > 0x7fff ? 0x7fff : pitch_);
        *(int16_t *)(buffer + 4) = htobe16(int16_t(pitch_));

        auto hdg_ = int32_t(hdg * 10);
        hdg_ = hdg_ < -3600 ? -3600 : (hdg_ > 3600 ? 3600 : hdg_);
        if (hdg_ < 0) hdg_ = (0x10000 + hdg_) & 0xffff;
        *(uint16_t *)(buffer + 6) = htobe16(uint16_t(hdg_ & 0x7fff) | ((mag ? 1 : 0) << 15));

        auto ias_ = uint16_t(ias < 0 ? 0 : (ias > 0x7fff ? 0x7fff : ias));
        *(uint16_t *)(buffer + 8) = htobe16(ias_);

        auto tas_ = uint16_t(tas < 0 ? 0 : (tas > 0x7fff ? 0x7fff : tas));
        *(uint16_t *)(buffer + 10) = htobe16(tas_);
        send_msg(buffer, sizeof(buffer));
    }

    void update_1hz() {
        float lat = XPLMGetDatad(lat_dref);
        float lng = XPLMGetDatad(lng_dref);
        float baro_alt = XPLMGetDataf(baro_alt_dref); // FIXME: use 29.92 as the reference
        float geo_alt = meter_to_feet(XPLMGetDatad(geo_alt_dref));
        float gs = XPLMGetDataf(gs_dref) * 1.94384;
        float vs = XPLMGetDataf(vs_dref);
        float trk = XPLMGetDataf(trk_dref);
        foreflight_addr_lock.lock();
        if (foreflight_addr.has_value()) {
            send_heartbeat(true);
            send_ownship_report(lat, lng, baro_alt,
                                std::optional(gs), vs, trk);
            send_ownship_geo_alt(geo_alt);
            send_idmsg();
            //print_debug("GDL90 frames sent lat=%.4f lng=%.4f baro_alt=%.0f geo_alt=%.0f gs=%.0f vs=%.0f trk=%0.f", lat, lng, baro_alt, geo_alt, gs, vs, trk);
        }
        foreflight_addr_lock.unlock();
    }

    void update_ahrs() {
        float roll = XPLMGetDataf(roll_dref);
        float pitch = XPLMGetDataf(pitch_dref);
        float hdg = XPLMGetDataf(hdg_dref);
        float ias = XPLMGetDataf(ias_dref);
        float tas = XPLMGetDataf(tas_dref);
        foreflight_addr_lock.lock();
        if (foreflight_addr.has_value()) {
            send_ahrs_message(roll, pitch, hdg, ias, tas);
        }
        foreflight_addr_lock.unlock();
    }
};

uint16_t GDL90::crc16_table[256];

XPlaneS *xps = nullptr;
GDL90 *gdl90 = nullptr;
XPLMFlightLoopID ctrl_loop_id;
XPLMFlightLoopID data_loop_id;
XPLMFlightLoopID gdl90_loop_id;
XPLMCommandRef ir_training_cmd;

float ctrl_loop(float elapsed, float, int, void *) {
    for (;;) {
        auto ret = xps->events.pop();
        if (!ret.has_value()) {
            break;
        }
        auto event = std::move(ret.value());
        event->handle();
    }
    return 0.01;
}

float data_loop(float, float, int, void *) {
    float elapsed = XPLMGetElapsedTime();
    std::unordered_map<SocketAddr, std::vector<std::pair<int32_t, std::shared_ptr<Subscription>>>> expired;
    while (!stops.empty() && elapsed + EPS > stops.front().first) {
        auto id = std::move(stops.front().second);
        stops.pop();
        expired[id.first].push_back(std::make_pair(id.second, id2sub[id]));
    }
    if (!expired.empty() && xps) {
        static const auto prefix = string("RREF,");

        for (const auto &[a, v]: expired) {
            std::vector<uint8_t> data_msg;
            data_msg.insert(data_msg.end(), prefix.begin(), prefix.end());
            data_msg.resize(prefix.length() + 8 * v.size());
            bytes_t ptr = &*data_msg.begin() + prefix.length();
            for (const auto &p: v) {
                *(uint32_t *)ptr = htole32(p.first);
                ptr += 4;
                float v = 0;
                const auto &sub = p.second;
                if (sub->idx.has_value()) {
                    auto i = sub->idx.value();
                    if (sub->datatype == xplmType_IntArray) {
                        int vv;
                        XPLMGetDatavi(sub->dataref, &vv, i, 1);
                        v = vv;
                    } else if (sub->datatype == xplmType_FloatArray) {
                        XPLMGetDatavf(sub->dataref, &v, i, 1);
                    }
                } else {
                    if (sub->datatype == xplmType_Int) {
                        v = XPLMGetDatai(sub->dataref);
                    } else if (sub->datatype == xplmType_Float) {
                        v = XPLMGetDataf(sub->dataref);
                    } else if (sub->datatype == xplmType_Double) {
                        v = XPLMGetDatad(sub->dataref);
                    }
                }
                uint32_t *vptr = (uint32_t *)&v;
                uint32_t encoded = htole32(*vptr);
                *(uint32_t *)ptr = encoded;
                ptr += 4;
            }
            xps->outbound(std::move(data_msg), a.get_sockaddr_in());
        }
    }
    if (stops.empty()) {
        // load the schedule for the next second.
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

int toggle_ir_training(XPLMCommandRef, XPLMCommandPhase phase, void *) {
    static float orig_cloud_data[9];
    if (phase != xplm_CommandBegin) {
        return 1;
    }
    XPLMCommandRef cmd_ref = XPLMFindCommand("sim/GPS/g1000n1_popup");
    XPLMCommandOnce(cmd_ref);
    cmd_ref = XPLMFindCommand("sim/GPS/g1000n3_popup");
    XPLMCommandOnce(cmd_ref);
    static const float cloud_ir[9] = {0.0, 10000.0, 11000.0, 10000.0, 11000.0, 21000.0, 6, 6, 6};
    static XPLMDataRef cloud_drefs[9];
    for (int i = 0; i < 3; i++) {
        static char buff[64];
        sprintf(buff, "sim/weather/cloud_base_msl_m[%d]", i);
        cloud_drefs[i] = XPLMFindDataRef(buff);
        sprintf(buff, "sim/weather/cloud_tops_msl_m[%d]", i);
        cloud_drefs[i + 3] = XPLMFindDataRef(buff);
        sprintf(buff, "sim/weather/cloud_coverage[%d]", i);
        cloud_drefs[i + 6] = XPLMFindDataRef(buff);
    }
    if (XPLMGetDataf(cloud_drefs[0]) < EPS) {
        print_debug("foggles off...");
        for (int i = 0; i < 9; i++) {
            XPLMSetDataf(cloud_drefs[i], orig_cloud_data[i]);
        }
    } else {
        print_debug("foggles on...");
        for (int i = 0; i < 9; i++) {
            orig_cloud_data[i] = XPLMGetDataf(cloud_drefs[i]);
            XPLMSetDataf(cloud_drefs[i], cloud_ir[i]);
        }
    }
    return 0;
}

struct GLWidget {
    virtual bool is_inside(float x, float y) { return false; }
    virtual void render(float x, float y, bool focused) = 0;
    virtual void on_keypress(char key, XPLMKeyFlags flag) {}
    virtual void on_mouse(XPLMMouseStatus status) {}
    virtual void on_focus() {}
};

XPLMWindowID teleport_window = 0;
std::vector<std::unique_ptr<GLWidget>> teleport_window_widgets;
ssize_t teleport_window_widgets_focused = -1;

struct GLTextInput: GLWidget {
    float l, r, t, b;
    string text;

    GLTextInput(float l, float b, float w, float h): \
        l(l), r(l + w), t(b + h), b(b), text("") {}
    bool is_inside(float x, float y) override {
        return l < x && x < r && b < y && y < t; 
    }
    virtual bool verify_input(string &edited) {
        return true;
    }
    void on_keypress(char key, XPLMKeyFlags flag) override {
        if (!(flag & xplm_DownFlag)) {
            return;
        }
        if (isprint(key)) {
            auto edited = text + key;
            if (verify_input(edited)) {
                text = edited;
            }
        } else if (key == '\b' && text.length() > 0) {
            text.resize(text.length() - 1);
        }
    }
    void render(float x, float y, bool focused) override {
        float col[3] = {1, 1, 1};
        if (!focused) {
            col[0] = 0.5;
            col[1] = 0.5;
            col[2] = 0.5;
        }
        glColor3fv(col);
        glRectf(x + l, y + t, x + r, y + b);
        float white[] = {0, 0, 0};
        XPLMDrawString(white, x + l + 5, y + t - 10, text.c_str(), nullptr, xplmFont_Proportional);
    }
};

struct GLFixInput: public GLTextInput {
    using GLTextInput::GLTextInput;

    bool verify_input(string &edited) override {
        std::transform(edited.begin(), edited.end(), edited.begin(), ::toupper);
        return std::regex_match(edited, std::regex("[A-Z0-9.]*"));
    }
};

struct GLAltInput: public GLTextInput {
    using GLTextInput::GLTextInput;

    bool verify_input(string &edited) override {
        return std::regex_match(edited, std::regex("[-0-9.]*"));
    }

    void on_focus() override {
        if (text.length() == 0) {
            text = to_string_with_precision(meter_to_feet(get_elevation()), 0);
        }
    }
};

struct GLGSInput: public GLTextInput {
    using GLTextInput::GLTextInput;

    bool verify_input(string &edited) override {
        return std::regex_match(edited, std::regex("[0-9.]*"));
    }

    void on_focus() override {
        if (text.length() == 0) {
            text = to_string_with_precision(std::max(0.0f, get_groundspeed()), 0);
        }
    }
};

struct GLButton: GLWidget {
    float l, r, t, b;
    string text;
    bool down;

    GLButton(float x0, float y0, const char *text, float w): l(x0), r(x0 + w), t(y0 + 15), b(y0), text(text), down(false) {}
    bool is_inside(float x, float y) override {
        return l < x && x < r && b < y && y < t; 
    }
    void on_mouse(XPLMMouseStatus status) override {
        switch (status) {
            case xplm_MouseDown: {
                down = true;
                string id = dynamic_cast<GLFixInput*>(teleport_window_widgets[1].get())->text;
                string alt = dynamic_cast<GLAltInput*>(teleport_window_widgets[3].get())->text;
                string gs = dynamic_cast<GLGSInput*>(teleport_window_widgets[5].get())->text;
                xps->events.push(std::unique_ptr<Event>(new TeleportEvent(id, alt, gs)));
            }
            break;
            case xplm_MouseUp: down = false; break;
            case xplm_MouseDrag: down = true; break;
        }
    }
    void render(float x, float y, bool) override {
        float col[3] = {0.4218, 0.5391, 0.9297};
        float white[] = {0, 0, 0};
        if (down) {
            for (auto &c: col) c = 1 - c;
            for (auto &w: white) w = 1 - w;
        }
        glColor3fv(col);
        glRectf(x + l, y + t, x + r, y + b);
        XPLMDrawString(white, x + l + 10, y + t - 10, text.c_str(), nullptr, xplmFont_Proportional);
    }
};


struct GLTextLabel: GLWidget {
    float x0, y0;
    string text;
    GLTextLabel(float x0, float y0, const char *text): x0(x0), y0(y0), text(text) {}
    void render(float x, float y, bool) override {
        float col[] = {1.0, 1.0, 1.0};
        XPLMDrawString(col, x0 + x, y0 + y, text.c_str(), nullptr, xplmFont_Proportional);
    }
};

int teleport_window_on_mouse_click(XPLMWindowID id, int x, int y, XPLMMouseStatus status, void *) {
    int l, t, r, b;
    XPLMGetWindowGeometry(id, &l, &t, &r, &b);

    if (status == xplm_MouseDown) {
        teleport_window_widgets_focused = -1;
    }
    for (size_t i = 0; i < teleport_window_widgets.size(); i++) {
        const auto &w = teleport_window_widgets[i];
        if (w->is_inside(x - l, y - t)) {
            if (status == xplm_MouseDown) {
                teleport_window_widgets_focused = i;
                w->on_focus();
            }
            w->on_mouse(status);
            XPLMTakeKeyboardFocus(id);
            return 1;
        }
    }
    return 0;
}

void teleport_window_on_keyboard(XPLMWindowID id, char key, XPLMKeyFlags flag, char inVirtualKey, void *, int losing_focus) {
    if (id != teleport_window || losing_focus) {
        return;
    }
    if (teleport_window_widgets_focused < 0) {
        return;
    }
    const auto &w = teleport_window_widgets[teleport_window_widgets_focused];
    w->on_keypress(key, flag);
}

void teleport_window_on_draw(XPLMWindowID id, void* refcon) {
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 1, 0);
    int l, t, r, b;
    XPLMGetWindowGeometry(id, &l, &t, &r, &b);
    for (size_t i = 0; i < teleport_window_widgets.size(); i++) {
        teleport_window_widgets[i]->render(l, t, ssize_t(i) == teleport_window_widgets_focused);
    }
}

void menu_handler(void *inMenuRef, void *inItemRef) {
    auto id = uintptr_t(inItemRef);
    switch (id) {
        case 0: {
            XPLMCommandRef cmd_ref = XPLMFindCommand("lfd/toggle_imc_foggles");
            XPLMCommandOnce(cmd_ref);
        }
        break;
        case 1: {
            if (!teleport_window) {
                teleport_window_widgets.push_back(std::make_unique<GLTextLabel>(10, -15, "Enter the fix:"));
                teleport_window_widgets.push_back(std::make_unique<GLFixInput>(100, -20, 100, 15));
                teleport_window_widgets.push_back(std::make_unique<GLTextLabel>(10, -35, "Altitude (ft.):"));
                teleport_window_widgets.push_back(std::make_unique<GLAltInput>(100, -40, 100, 15));
                teleport_window_widgets.push_back(std::make_unique<GLTextLabel>(10, -55, "GS (kts.):"));
                teleport_window_widgets.push_back(std::make_unique<GLGSInput>(100, -60, 100, 15));
                teleport_window_widgets.push_back(std::make_unique<GLButton>(10, -100, "Go!", 50));

                int x0 = 500, y0 = 500;
                int w = 300, h = 150;
                XPLMCreateWindow_t win;
                win.structSize = sizeof(win);
                win.left = x0;
                win.bottom = y0 - h;
                win.right = x0 + w;
                win.top = y0;
                win.visible = 1;
                win.drawWindowFunc = teleport_window_on_draw;
                win.handleMouseClickFunc = teleport_window_on_mouse_click;
                win.handleRightClickFunc = nullptr;
                win.handleMouseWheelFunc = nullptr;
                win.handleKeyFunc = teleport_window_on_keyboard;
                win.handleCursorFunc = nullptr;
                win.refcon = nullptr;
                win.layer = xplm_WindowLayerFloatingWindows;
                teleport_window = XPLMCreateWindowEx(&win);
                XPLMSetWindowPositioningMode(teleport_window, xplm_WindowPositionFree, -1);
                XPLMSetWindowResizingLimits(teleport_window, w, h, w, h);
                XPLMSetWindowTitle(teleport_window, "Teleport To");
            } else {
                XPLMSetWindowIsVisible(teleport_window, 1);
            }
        }
        break;
        case 2: XPLMReloadPlugins();
        break;
    }
}

XPLMMenuID menu_id;

PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc) {
    strcpy(outName, "Loupe Flightdeck");
    strcpy(outSig, "com.determinant.loupeflightdeck");
    strcpy(outDesc, "A plugin that reimplements X-Plane's basic UDP API and GDL90 support.");

    auto item = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "Loupe Flightdeck", 0, 1);

    menu_id = XPLMCreateMenu(
            "Loupe Flightdeck",
            XPLMFindPluginsMenu(),
            item,
            menu_handler,
            0);

    XPLMAppendMenuItem(menu_id, "Toggle Foggles", (void *)0, 1);
    XPLMAppendMenuItem(menu_id, "Teleport to Fix", (void *)1, 1);
    XPLMAppendMenuItem(menu_id, "Reload Plugins", (void *)2, 1);
    return 1;
}

PLUGIN_API void	XPluginStop() {
    XPLMDestroyMenu(menu_id);
}

PLUGIN_API int XPluginEnable() {
    GDL90::crc_init();

    xps = new XPlaneS();
    gdl90 = new GDL90();

    XPLMCreateFlightLoop_t loop_cfg;
    loop_cfg.structSize = sizeof(XPLMCreateFlightLoop_t);
    loop_cfg.phase = xplm_FlightLoop_Phase_AfterFlightModel;
    loop_cfg.refcon = nullptr;
    loop_cfg.callbackFunc = ctrl_loop;
    ctrl_loop_id = XPLMCreateFlightLoop(&loop_cfg);
    XPLMScheduleFlightLoop(ctrl_loop_id, -1, 1);

    loop_cfg.callbackFunc = data_loop;
    data_loop_id = XPLMCreateFlightLoop(&loop_cfg);
    XPLMScheduleFlightLoop(data_loop_id, -1, 1);

    loop_cfg.callbackFunc = gdl90_loop;
    gdl90_loop_id = XPLMCreateFlightLoop(&loop_cfg);
    XPLMScheduleFlightLoop(gdl90_loop_id, -1, 1);

    ir_training_cmd = XPLMCreateCommand(
            "lfd/toggle_imc_foggles",
            "Toggle IR training mode. This will toggle the outside vision (IMC and back to the original condition) and also toggle the G1000 PFD/MFD display.");
    XPLMRegisterCommandHandler(ir_training_cmd, toggle_ir_training, false, nullptr);
    return 1;
}

PLUGIN_API void XPluginDisable() {
    XPLMUnregisterCommandHandler(ir_training_cmd, toggle_ir_training, false, nullptr);
    XPLMDestroyFlightLoop(ctrl_loop_id);
    XPLMDestroyFlightLoop(data_loop_id);
    XPLMDestroyFlightLoop(gdl90_loop_id);

    if (teleport_window) {
        XPLMDestroyWindow(teleport_window);
        teleport_window = 0;
    }

    if (xps) {
        delete xps;
        xps = nullptr;
    } else {
        print_debug("XP server should not be nullptr!");
    }
    print_debug("XPS is disabled");
    if (gdl90) {
        delete gdl90;
        gdl90 = nullptr;
    } else {
        print_debug("GDL90 server should not be nullptr!");
    }
    print_debug("GDL90 is disabled");
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int msg, void*) {}
