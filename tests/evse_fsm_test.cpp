// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <fsm/specialization/sync/simple.hpp>

#include "../tools/evse/evse_fsm.hpp"

namespace {

// Use the real SlacIO with the loopback interface for testing

const uint8_t SAMPLE_NMK[] = {0x34, 0x52, 0x23, 0x54, 0x45, 0xae, 0xf2, 0xd4,
                              0x55, 0xfe, 0xff, 0x31, 0xa3, 0xb3, 0x03, 0xad};

const uint8_t EV_MAC[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x02};
const uint8_t EVSE_MAC[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x03};

void set_src(slac::messages::HomeplugMessage& msg, const uint8_t mac[ETH_ALEN]) {
    memcpy(msg.get_src_mac(), mac, ETH_ALEN);
}

TEST(EvseFSM, complete_handshake) {
    SlacIO io("lo");
    EvseFSM fsm(io);
    fsm.set_nmk(SAMPLE_NMK);

    fsm::sync::PosixController<EvseFSM::StateHandleType> ctrl;
    ctrl.reset(fsm.sd_reset);

    ctrl.feed(); // send CM_SET_KEY.REQ

    slac::messages::HomeplugMessage msg;
    slac::messages::cm_set_key_cnf set_key_cnf{};
    set_key_cnf.result = slac::defs::CM_SET_KEY_CNF_RESULT_SUCCESS;
    msg.setup_payload(&set_key_cnf, sizeof(set_key_cnf), slac::defs::MMTYPE_CM_SET_KEY | slac::defs::MMTYPE_MODE_CNF,
                      slac::defs::MMV::AV_1_0);
    msg.setup_ethernet_header(EVSE_MAC, EV_MAC);
    ctrl.submit_event(EventSlacMessage(msg));
    ctrl.feed();
    EXPECT_EQ(ctrl.current_state()->id.id, State::Idle);

    ctrl.submit_event(EventEnterBCD());
    ctrl.feed();
    EXPECT_EQ(ctrl.current_state()->id.id, State::WaitForMatchingStart);

    slac::messages::cm_slac_parm_req parm_req{};
    parm_req.application_type = slac::defs::COMMON_APPLICATION_TYPE;
    parm_req.security_type = slac::defs::COMMON_SECURITY_TYPE;
    uint8_t run_id[slac::defs::RUN_ID_LEN];
    for (int i = 0; i < slac::defs::RUN_ID_LEN; ++i) {
        run_id[i] = static_cast<uint8_t>(i + 1);
    }
    memcpy(parm_req.run_id, run_id, sizeof(run_id));
    msg.setup_payload(&parm_req, sizeof(parm_req), slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_REQ,
                      slac::defs::MMV::AV_1_0);
    msg.setup_ethernet_header(EVSE_MAC, EV_MAC);
    ctrl.submit_event(EventSlacMessage(msg));
    ctrl.feed();
    ctrl.feed();
    EXPECT_EQ(ctrl.current_state()->id.id, State::Matching);

    slac::messages::cm_start_atten_char_ind start{};
    start.application_type = slac::defs::COMMON_APPLICATION_TYPE;
    start.security_type = slac::defs::COMMON_SECURITY_TYPE;
    start.num_sounds = slac::defs::CM_SLAC_PARM_CNF_NUM_SOUNDS;
    start.timeout = slac::defs::CM_SLAC_PARM_CNF_TIMEOUT;
    start.resp_type = slac::defs::CM_SLAC_PARM_CNF_RESP_TYPE;
    memcpy(start.forwarding_sta, EV_MAC, ETH_ALEN);
    memcpy(start.run_id, run_id, sizeof(run_id));
    msg.setup_payload(&start, sizeof(start), slac::defs::MMTYPE_CM_START_ATTEN_CHAR | slac::defs::MMTYPE_MODE_IND,
                      slac::defs::MMV::AV_1_0);
    msg.setup_ethernet_header(EVSE_MAC, EV_MAC);
    ctrl.submit_event(EventSlacMessage(msg));
    ctrl.feed();
    EXPECT_EQ(ctrl.current_state()->id.id, State::Sounding);

    for (int i = 0; i < slac::defs::CM_SLAC_PARM_CNF_NUM_SOUNDS; ++i) {
        slac::messages::cm_mnbc_sound_ind sound{};
        sound.application_type = slac::defs::COMMON_APPLICATION_TYPE;
        sound.security_type = slac::defs::COMMON_SECURITY_TYPE;
        memset(sound.sender_id, 0, sizeof(sound.sender_id));
        sound.remaining_sound_count = static_cast<uint8_t>(slac::defs::CM_SLAC_PARM_CNF_NUM_SOUNDS - i - 1);
        memcpy(sound.run_id, run_id, sizeof(run_id));
        memset(sound.random, 0, sizeof(sound.random));
        msg.setup_payload(&sound, sizeof(sound), slac::defs::MMTYPE_CM_MNBC_SOUND | slac::defs::MMTYPE_MODE_IND,
                          slac::defs::MMV::AV_1_0);
        msg.setup_ethernet_header(EVSE_MAC, EV_MAC);
        ctrl.submit_event(EventSlacMessage(msg));
        ctrl.feed();

        slac::messages::cm_atten_profile_ind profile{};
        memcpy(profile.pev_mac, EV_MAC, ETH_ALEN);
        profile.num_groups = slac::defs::AAG_LIST_LEN;
        profile._reserved = 0;
        memset(profile.aag, 1, sizeof(profile.aag));
        msg.setup_payload(&profile, sizeof(profile), slac::defs::MMTYPE_CM_ATTEN_PROFILE | slac::defs::MMTYPE_MODE_IND,
                          slac::defs::MMV::AV_1_0);
        msg.setup_ethernet_header(EVSE_MAC, EV_MAC);
        ctrl.submit_event(EventSlacMessage(msg));
        ctrl.feed();
    }

    // FinishSounding event is generated internally after last sound
    ctrl.feed();
    EXPECT_EQ(ctrl.current_state()->id.id, State::DoAttenChar);

    slac::messages::cm_atten_char_rsp rsp{};
    rsp.application_type = slac::defs::COMMON_APPLICATION_TYPE;
    rsp.security_type = slac::defs::COMMON_SECURITY_TYPE;
    memcpy(rsp.source_address, EV_MAC, ETH_ALEN);
    memcpy(rsp.run_id, run_id, sizeof(run_id));
    memset(rsp.source_id, 0, sizeof(rsp.source_id));
    memset(rsp.resp_id, 0, sizeof(rsp.resp_id));
    rsp.result = slac::defs::CM_ATTEN_CHAR_RSP_RESULT;
    msg.setup_payload(&rsp, sizeof(rsp), slac::defs::MMTYPE_CM_ATTEN_CHAR | slac::defs::MMTYPE_MODE_RSP,
                      slac::defs::MMV::AV_1_0);
    msg.setup_ethernet_header(EVSE_MAC, EV_MAC);
    ctrl.submit_event(EventSlacMessage(msg));
    ctrl.feed();
    EXPECT_EQ(ctrl.current_state()->id.id, State::WaitForSlacMatch);

    slac::messages::cm_slac_match_req match_req{};
    match_req.application_type = slac::defs::COMMON_APPLICATION_TYPE;
    match_req.security_type = slac::defs::COMMON_SECURITY_TYPE;
    match_req.mvf_length = htole16(slac::defs::CM_SLAC_MATCH_REQ_MVF_LENGTH);
    memset(match_req.pev_id, 0, sizeof(match_req.pev_id));
    memcpy(match_req.pev_mac, EV_MAC, ETH_ALEN);
    memset(match_req.evse_id, 0, sizeof(match_req.evse_id));
    memcpy(match_req.evse_mac, EVSE_MAC, ETH_ALEN);
    memcpy(match_req.run_id, run_id, sizeof(run_id));

    msg.setup_payload(&match_req, sizeof(match_req), slac::defs::MMTYPE_CM_SLAC_MATCH | slac::defs::MMTYPE_MODE_REQ,
                      slac::defs::MMV::AV_1_0);
    msg.setup_ethernet_header(EVSE_MAC, EV_MAC);
    ctrl.submit_event(EventSlacMessage(msg));
    ctrl.feed();

    ctrl.submit_event(EventLinkDetected());
    ctrl.feed();

    EXPECT_EQ(ctrl.current_state()->id.id, State::Matched);
}

} // namespace
