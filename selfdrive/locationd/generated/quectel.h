#ifndef QUECTEL_H_
#define QUECTEL_H_

// This is a generated file! Please edit source .ksy file and use kaitai-struct-compiler to rebuild

#include "kaitai/kaitaistruct.h"
#include <stdint.h>

#if KAITAI_STRUCT_VERSION < 9000L
#error "Incompatible Kaitai Struct C++/STL API: version 0.9 or later is required"
#endif

/**
 * L26-DR is a multi-constellation GNSS module supporting Dead Reckoning (DR) 
 * function. The module supports GPS, GLONASS, BeiDou, Galileo and QZSS 
 * constellations and features accurate acquisition.
 * 
 * All of the supported constellations cannot be enabled at the same time, 
 * the allowed combinations to achieve maximum coverage are: 
 * GPS + Galileo + QZSS + GLONASS and GPS + Galileo + QZSS + BeiDou. 
 * Any constellation can be enabled as standalone satellite navigation system. 
 * Thus, the module is suitable for applications such as position fixing and 
 * navigation.
 * 
 * This document describes the software aspects of L26-DR. L26-DR supports 
 * NMEA 0183 standard and proprietary messages to report GNSS information. 
 * Also, it supports to control and configure the module through proprietary 
 * commands.
 * \sa L26-DR_GNSS_Protocol_Specification_V1.1
 */

class quectel_t : public kaitai::kstruct {

public:
    class pstmdrgps_t;
    class pstmdrstep_t;
    class pstmdrconfid_t;
    class message_t;
    class pstmdrstype_t;
    class pstmdrsenmsg_t;
    class pstmdrdebug_t;
    class pstmdrstate_t;

    quectel_t(kaitai::kstream* p__io, kaitai::kstruct* p__parent = 0, quectel_t* p__root = 0);

private:
    void _read();
    void _clean_up();

public:
    ~quectel_t();

    class pstmdrgps_t : public kaitai::kstruct {

    public:

        pstmdrgps_t(kaitai::kstream* p__io, quectel_t::message_t* p__parent = 0, quectel_t* p__root = 0);

    private:
        void _read();
        void _clean_up();

    public:
        ~pstmdrgps_t();

    private:
        std::string m_lat;
        std::string m_lon;
        std::string m_vn;
        std::string m_ve;
        std::string m_pdop;
        std::string m_hdop;
        std::string m_vdop;
        std::string m_rms_pos_residual;
        std::string m_rms_vel_residual;
        std::string m_vv;
        std::string m_height;
        quectel_t* m__root;
        quectel_t::message_t* m__parent;

    public:
        std::string lat() const { return m_lat; }
        std::string lon() const { return m_lon; }
        std::string vn() const { return m_vn; }
        std::string ve() const { return m_ve; }
        std::string pdop() const { return m_pdop; }
        std::string hdop() const { return m_hdop; }
        std::string vdop() const { return m_vdop; }
        std::string rms_pos_residual() const { return m_rms_pos_residual; }
        std::string rms_vel_residual() const { return m_rms_vel_residual; }
        std::string vv() const { return m_vv; }
        std::string height() const { return m_height; }
        quectel_t* _root() const { return m__root; }
        quectel_t::message_t* _parent() const { return m__parent; }
    };

    class pstmdrstep_t : public kaitai::kstruct {

    public:

        pstmdrstep_t(kaitai::kstream* p__io, quectel_t::message_t* p__parent = 0, quectel_t* p__root = 0);

    private:
        void _read();
        void _clean_up();

    public:
        ~pstmdrstep_t();

    private:
        std::string m_sample_count;
        std::string m_ave_gyro;
        std::string m_gyro_noise;
        std::string m_odo_pulse;
        std::string m_reserved1;
        std::string m_delta_t;
        std::string m_reserved2;
        std::string m_valid_odo;
        quectel_t* m__root;
        quectel_t::message_t* m__parent;

    public:
        std::string sample_count() const { return m_sample_count; }
        std::string ave_gyro() const { return m_ave_gyro; }
        std::string gyro_noise() const { return m_gyro_noise; }
        std::string odo_pulse() const { return m_odo_pulse; }
        std::string reserved1() const { return m_reserved1; }
        std::string delta_t() const { return m_delta_t; }
        std::string reserved2() const { return m_reserved2; }
        std::string valid_odo() const { return m_valid_odo; }
        quectel_t* _root() const { return m__root; }
        quectel_t::message_t* _parent() const { return m__parent; }
    };

    class pstmdrconfid_t : public kaitai::kstruct {

    public:

        pstmdrconfid_t(kaitai::kstream* p__io, quectel_t::message_t* p__parent = 0, quectel_t* p__root = 0);

    private:
        void _read();
        void _clean_up();

    public:
        ~pstmdrconfid_t();

    private:
        std::string m_lat_std_dev;
        std::string m_long_std_dev;
        std::string m_heading_std_dev;
        std::string m_reserved1;
        std::string m_gyro_bias_std_dev;
        std::string m_odo_scale_std_dev;
        std::string m_reserved;
        std::string m_acc_offset_std_dev;
        std::string m_height_std_dev;
        std::string m_maj_semi_axis;
        std::string m_minor_semi_axis;
        std::string m_ellipse_angle;
        std::string m_speed_std_dev;
        quectel_t* m__root;
        quectel_t::message_t* m__parent;

    public:
        std::string lat_std_dev() const { return m_lat_std_dev; }
        std::string long_std_dev() const { return m_long_std_dev; }
        std::string heading_std_dev() const { return m_heading_std_dev; }
        std::string reserved1() const { return m_reserved1; }
        std::string gyro_bias_std_dev() const { return m_gyro_bias_std_dev; }
        std::string odo_scale_std_dev() const { return m_odo_scale_std_dev; }
        std::string reserved() const { return m_reserved; }
        std::string acc_offset_std_dev() const { return m_acc_offset_std_dev; }
        std::string height_std_dev() const { return m_height_std_dev; }
        std::string maj_semi_axis() const { return m_maj_semi_axis; }
        std::string minor_semi_axis() const { return m_minor_semi_axis; }
        std::string ellipse_angle() const { return m_ellipse_angle; }
        std::string speed_std_dev() const { return m_speed_std_dev; }
        quectel_t* _root() const { return m__root; }
        quectel_t::message_t* _parent() const { return m__parent; }
    };

    class message_t : public kaitai::kstruct {

    public:

        message_t(kaitai::kstream* p__io, quectel_t* p__parent = 0, quectel_t* p__root = 0);

    private:
        void _read();
        void _clean_up();

    public:
        ~message_t();

    private:
        uint8_t m_start_delimiter;
        std::string m_talker_id;
        std::string m_sentence;
        kaitai::kstruct* m_dat;
        bool n_dat;

    public:
        bool _is_null_dat() { dat(); return n_dat; };

    private:
        std::string m_checksum;
        quectel_t* m__root;
        quectel_t* m__parent;

    public:
        uint8_t start_delimiter() const { return m_start_delimiter; }
        std::string talker_id() const { return m_talker_id; }
        std::string sentence() const { return m_sentence; }
        kaitai::kstruct* dat() const { return m_dat; }
        std::string checksum() const { return m_checksum; }
        quectel_t* _root() const { return m__root; }
        quectel_t* _parent() const { return m__parent; }
    };

    class pstmdrstype_t : public kaitai::kstruct {

    public:

        pstmdrstype_t(kaitai::kstream* p__io, quectel_t::message_t* p__parent = 0, quectel_t* p__root = 0);

    private:
        void _read();
        void _clean_up();

    public:
        ~pstmdrstype_t();

    private:
        std::string m_sensor_type;
        quectel_t* m__root;
        quectel_t::message_t* m__parent;

    public:
        std::string sensor_type() const { return m_sensor_type; }
        quectel_t* _root() const { return m__root; }
        quectel_t::message_t* _parent() const { return m__parent; }
    };

    class pstmdrsenmsg_t : public kaitai::kstruct {

    public:

        pstmdrsenmsg_t(kaitai::kstream* p__io, quectel_t::message_t* p__parent = 0, quectel_t* p__root = 0);

    private:
        void _read();
        void _clean_up();

    public:
        ~pstmdrsenmsg_t();

    private:
        uint8_t m_id_msg;
        std::string m_cpu_timestamp;
        std::string m_odo;
        bool n_odo;

    public:
        bool _is_null_odo() { odo(); return n_odo; };

    private:
        std::string m_dir;
        bool n_dir;

    public:
        bool _is_null_dir() { dir(); return n_dir; };

    private:
        std::string m_speed;
        bool n_speed;

    public:
        bool _is_null_speed() { speed(); return n_speed; };

    private:
        std::string m_x;
        bool n_x;

    public:
        bool _is_null_x() { x(); return n_x; };

    private:
        std::string m_y;
        bool n_y;

    public:
        bool _is_null_y() { y(); return n_y; };

    private:
        std::string m_z;
        bool n_z;

    public:
        bool _is_null_z() { z(); return n_z; };

    private:
        quectel_t* m__root;
        quectel_t::message_t* m__parent;

    public:
        uint8_t id_msg() const { return m_id_msg; }
        std::string cpu_timestamp() const { return m_cpu_timestamp; }
        std::string odo() const { return m_odo; }
        std::string dir() const { return m_dir; }
        std::string speed() const { return m_speed; }
        std::string x() const { return m_x; }
        std::string y() const { return m_y; }
        std::string z() const { return m_z; }
        quectel_t* _root() const { return m__root; }
        quectel_t::message_t* _parent() const { return m__parent; }
    };

    class pstmdrdebug_t : public kaitai::kstruct {

    public:

        pstmdrdebug_t(kaitai::kstream* p__io, quectel_t::message_t* p__parent = 0, quectel_t* p__root = 0);

    private:
        void _read();
        void _clean_up();

    public:
        ~pstmdrdebug_t();

    private:
        std::string m_lat_err;
        std::string m_lon_err;
        std::string m_head_err;
        std::string m_speed_err;
        std::string m_height_err;
        std::string m_vv_err;
        quectel_t* m__root;
        quectel_t::message_t* m__parent;

    public:
        std::string lat_err() const { return m_lat_err; }
        std::string lon_err() const { return m_lon_err; }
        std::string head_err() const { return m_head_err; }
        std::string speed_err() const { return m_speed_err; }
        std::string height_err() const { return m_height_err; }
        std::string vv_err() const { return m_vv_err; }
        quectel_t* _root() const { return m__root; }
        quectel_t::message_t* _parent() const { return m__parent; }
    };

    class pstmdrstate_t : public kaitai::kstruct {

    public:

        pstmdrstate_t(kaitai::kstream* p__io, quectel_t::message_t* p__parent = 0, quectel_t* p__root = 0);

    private:
        void _read();
        void _clean_up();

    public:
        ~pstmdrstate_t();

    private:
        std::string m_lat;
        std::string m_lon;
        std::string m_heading;
        std::string m_speed;
        std::string m_gyro_bias;
        std::string m_gyro_sensitivity;
        std::string m_odo_scale;
        std::string m_reserved;
        std::string m_acc_bias;
        std::string m_height;
        quectel_t* m__root;
        quectel_t::message_t* m__parent;

    public:
        std::string lat() const { return m_lat; }
        std::string lon() const { return m_lon; }
        std::string heading() const { return m_heading; }
        std::string speed() const { return m_speed; }
        std::string gyro_bias() const { return m_gyro_bias; }
        std::string gyro_sensitivity() const { return m_gyro_sensitivity; }
        std::string odo_scale() const { return m_odo_scale; }
        std::string reserved() const { return m_reserved; }
        std::string acc_bias() const { return m_acc_bias; }
        std::string height() const { return m_height; }
        quectel_t* _root() const { return m__root; }
        quectel_t::message_t* _parent() const { return m__parent; }
    };

private:
    message_t* m_messages;
    quectel_t* m__root;
    kaitai::kstruct* m__parent;

public:
    message_t* messages() const { return m_messages; }
    quectel_t* _root() const { return m__root; }
    kaitai::kstruct* _parent() const { return m__parent; }
};

#endif  // QUECTEL_H_

