#include <gtest/gtest.h>
#include <fsm/fsm.hpp>
#include <fsm/buffer.hpp>

namespace {

enum class E { Go };

using Buffer = fsm::buffer::SwapBuffer<64,64,1>;
using FSMType = fsm::FSM<E, int, Buffer>;
using Alloc = FSMType::StateAllocatorType;
using SimpleBase = FSMType::SimpleStateType;

struct Next : public SimpleBase {
    fsm::states::HandleEventResult handle_event(Alloc&, E) override {
        return Alloc::PASS_ON;
    }
};

struct Start : public SimpleBase {
    fsm::states::HandleEventResult handle_event(Alloc& alloc, E) override {
        return alloc.create_simple<Next>();
    }
};

}

TEST(FSMBuffer, SimpleTransition) {
    Buffer buf{};
    FSMType fsm(buf);
    fsm.reset<Start>();
    EXPECT_EQ(fsm.handle_event(E::Go), fsm::HandleEventResult::SUCCESS);
}
