// This is a generated file! Please edit source .ksy file and use kaitai-struct-compiler to rebuild

#include "quectel.h"

quectel_t::quectel_t(kaitai::kstream* p__io, kaitai::kstruct* p__parent, quectel_t* p__root) : kaitai::kstruct(p__io) {
    m__parent = p__parent;
    m__root = this;
    m_messages = 0;

    try {
        _read();
    } catch(...) {
        _clean_up();
        throw;
    }
}

void quectel_t::_read() {
    m_messages = new message_t(m__io, this, m__root);
}

quectel_t::~quectel_t() {
    _clean_up();
}

void quectel_t::_clean_up() {
    if (m_messages) {
        delete m_messages; m_messages = 0;
    }
}

quectel_t::pstmdrgps_t::pstmdrgps_t(kaitai::kstream* p__io, quectel_t::message_t* p__parent, quectel_t* p__root) : kaitai::kstruct(p__io) {
    m__parent = p__parent;
    m__root = p__root;

    try {
        _read();
    } catch(...) {
        _clean_up();
        throw;
    }
}

void quectel_t::pstmdrgps_t::_read() {
    m_lat = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_lon = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_vn = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_ve = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_pdop = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_hdop = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_vdop = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_rms_pos_residual = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_rms_vel_residual = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_vv = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_height = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(42, false, true, true), std::string("ASCII"));
}

quectel_t::pstmdrgps_t::~pstmdrgps_t() {
    _clean_up();
}

void quectel_t::pstmdrgps_t::_clean_up() {
}

quectel_t::pstmdrstep_t::pstmdrstep_t(kaitai::kstream* p__io, quectel_t::message_t* p__parent, quectel_t* p__root) : kaitai::kstruct(p__io) {
    m__parent = p__parent;
    m__root = p__root;

    try {
        _read();
    } catch(...) {
        _clean_up();
        throw;
    }
}

void quectel_t::pstmdrstep_t::_read() {
    m_sample_count = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_ave_gyro = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_gyro_noise = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_odo_pulse = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_reserved1 = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_delta_t = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_reserved2 = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_valid_odo = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(42, false, true, true), std::string("ASCII"));
}

quectel_t::pstmdrstep_t::~pstmdrstep_t() {
    _clean_up();
}

void quectel_t::pstmdrstep_t::_clean_up() {
}

quectel_t::pstmdrconfid_t::pstmdrconfid_t(kaitai::kstream* p__io, quectel_t::message_t* p__parent, quectel_t* p__root) : kaitai::kstruct(p__io) {
    m__parent = p__parent;
    m__root = p__root;

    try {
        _read();
    } catch(...) {
        _clean_up();
        throw;
    }
}

void quectel_t::pstmdrconfid_t::_read() {
    m_lat_std_dev = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_long_std_dev = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_heading_std_dev = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_reserved1 = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_gyro_bias_std_dev = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_odo_scale_std_dev = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_reserved = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_acc_offset_std_dev = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_height_std_dev = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_maj_semi_axis = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_minor_semi_axis = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_ellipse_angle = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_speed_std_dev = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(42, false, true, true), std::string("ASCII"));
}

quectel_t::pstmdrconfid_t::~pstmdrconfid_t() {
    _clean_up();
}

void quectel_t::pstmdrconfid_t::_clean_up() {
}

quectel_t::message_t::message_t(kaitai::kstream* p__io, quectel_t* p__parent, quectel_t* p__root) : kaitai::kstruct(p__io) {
    m__parent = p__parent;
    m__root = p__root;

    try {
        _read();
    } catch(...) {
        _clean_up();
        throw;
    }
}

void quectel_t::message_t::_read() {
    m_start_delimiter = m__io->read_u1();
    m_talker_id = kaitai::kstream::bytes_to_str(m__io->read_bytes(2), std::string("ASCII"));
    m_sentence = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    n_dat = true;
    {
        std::string on = sentence();
        if (on == std::string("MDRSTATE")) {
            n_dat = false;
            m_dat = new pstmdrstate_t(m__io, this, m__root);
        }
        else if (on == std::string("MDRSTYPE")) {
            n_dat = false;
            m_dat = new pstmdrstype_t(m__io, this, m__root);
        }
        else if (on == std::string("MDRCONFID")) {
            n_dat = false;
            m_dat = new pstmdrconfid_t(m__io, this, m__root);
        }
        else if (on == std::string("MDRSENMSG")) {
            n_dat = false;
            m_dat = new pstmdrsenmsg_t(m__io, this, m__root);
        }
        else if (on == std::string("MDRSTEP")) {
            n_dat = false;
            m_dat = new pstmdrstep_t(m__io, this, m__root);
        }
        else if (on == std::string("MDRGPS")) {
            n_dat = false;
            m_dat = new pstmdrgps_t(m__io, this, m__root);
        }
        else if (on == std::string("MDRDEBUG")) {
            n_dat = false;
            m_dat = new pstmdrdebug_t(m__io, this, m__root);
        }
    }
    m_checksum = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(10, false, true, true), std::string("ASCII"));
}

quectel_t::message_t::~message_t() {
    _clean_up();
}

void quectel_t::message_t::_clean_up() {
    if (!n_dat) {
        if (m_dat) {
            delete m_dat; m_dat = 0;
        }
    }
}

quectel_t::pstmdrstype_t::pstmdrstype_t(kaitai::kstream* p__io, quectel_t::message_t* p__parent, quectel_t* p__root) : kaitai::kstruct(p__io) {
    m__parent = p__parent;
    m__root = p__root;

    try {
        _read();
    } catch(...) {
        _clean_up();
        throw;
    }
}

void quectel_t::pstmdrstype_t::_read() {
    m_sensor_type = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(42, false, true, true), std::string("ASCII"));
}

quectel_t::pstmdrstype_t::~pstmdrstype_t() {
    _clean_up();
}

void quectel_t::pstmdrstype_t::_clean_up() {
}

quectel_t::pstmdrsenmsg_t::pstmdrsenmsg_t(kaitai::kstream* p__io, quectel_t::message_t* p__parent, quectel_t* p__root) : kaitai::kstruct(p__io) {
    m__parent = p__parent;
    m__root = p__root;

    try {
        _read();
    } catch(...) {
        _clean_up();
        throw;
    }
}

void quectel_t::pstmdrsenmsg_t::_read() {
    m_id_msg = m__io->read_u1();
    m_cpu_timestamp = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    n_odo = true;
    if (id_msg() == 3) {
        n_odo = false;
        m_odo = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    }
    n_dir = true;
    if (id_msg() == 3) {
        n_dir = false;
        m_dir = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(42, false, true, true), std::string("ASCII"));
    }
    n_speed = true;
    if (id_msg() == 14) {
        n_speed = false;
        m_speed = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(42, false, true, true), std::string("ASCII"));
    }
    n_x = true;
    if (id_msg() == 30) {
        n_x = false;
        m_x = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    }
    n_y = true;
    if (id_msg() == 30) {
        n_y = false;
        m_y = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    }
    n_z = true;
    if (id_msg() == 30) {
        n_z = false;
        m_z = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(42, false, true, true), std::string("ASCII"));
    }
}

quectel_t::pstmdrsenmsg_t::~pstmdrsenmsg_t() {
    _clean_up();
}

void quectel_t::pstmdrsenmsg_t::_clean_up() {
    if (!n_odo) {
    }
    if (!n_dir) {
    }
    if (!n_speed) {
    }
    if (!n_x) {
    }
    if (!n_y) {
    }
    if (!n_z) {
    }
}

quectel_t::pstmdrdebug_t::pstmdrdebug_t(kaitai::kstream* p__io, quectel_t::message_t* p__parent, quectel_t* p__root) : kaitai::kstruct(p__io) {
    m__parent = p__parent;
    m__root = p__root;

    try {
        _read();
    } catch(...) {
        _clean_up();
        throw;
    }
}

void quectel_t::pstmdrdebug_t::_read() {
    m_lat_err = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_lon_err = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_head_err = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_speed_err = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_height_err = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_vv_err = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(42, false, true, true), std::string("ASCII"));
}

quectel_t::pstmdrdebug_t::~pstmdrdebug_t() {
    _clean_up();
}

void quectel_t::pstmdrdebug_t::_clean_up() {
}

quectel_t::pstmdrstate_t::pstmdrstate_t(kaitai::kstream* p__io, quectel_t::message_t* p__parent, quectel_t* p__root) : kaitai::kstruct(p__io) {
    m__parent = p__parent;
    m__root = p__root;

    try {
        _read();
    } catch(...) {
        _clean_up();
        throw;
    }
}

void quectel_t::pstmdrstate_t::_read() {
    m_lat = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_lon = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_heading = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_speed = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_gyro_bias = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_gyro_sensitivity = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_odo_scale = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_reserved = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_acc_bias = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(44, false, true, true), std::string("ASCII"));
    m_height = kaitai::kstream::bytes_to_str(m__io->read_bytes_term(42, false, true, true), std::string("ASCII"));
}

quectel_t::pstmdrstate_t::~pstmdrstate_t() {
    _clean_up();
}

void quectel_t::pstmdrstate_t::_clean_up() {
}

