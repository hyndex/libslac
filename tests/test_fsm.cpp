#include <gtest/gtest.h>
#include <fsm/fsm.hpp>
#include <fsm/buffer.hpp>

namespace {

enum class Ev { Add, Unknown };

struct Compound;

using Buffer = fsm::buffer::SwapBuffer<64, 64, 1>;
using FSMType = fsm::FSM<Ev, int, Buffer>;
using Allocator = FSMType::StateAllocatorType;
using SimpleBase = FSMType::SimpleStateType;
using CompoundBase = FSMType::CompoundStateType;

struct Simple : public SimpleBase {
    fsm::states::HandleEventResult handle_event(Allocator& alloc, Ev ev) override {
        if (ev == Ev::Add) {
            alloc.create_compound<Compound>();
            return alloc.create_simple<Simple>();
        }
        return Allocator::PASS_ON;
    }
};

struct Compound : public CompoundBase {
    fsm::states::HandleEventResult handle_event(Allocator&, Ev) override { return Allocator::PASS_ON; }
};

} // namespace

TEST(StateAllocator, Overflow) {
    Buffer buf{};
    fsm::_impl::StateAllocator<Buffer> alloc(buf);
    alloc.make_ready_for_nesting_level(Buffer::MAX_NESTING_LEVEL);
    bool ok = alloc.create_compound<Compound>();
    EXPECT_FALSE(ok);
    EXPECT_EQ(alloc.get_internal_state(), fsm::_impl::StateAllocator<Buffer>::InternalState::FAILED_COMPOUND_OVERFLOW);
}

TEST(FSM, UnhandledPassThrough) {
    Buffer buf{};
    FSMType fsm(buf);
    fsm.reset<Simple>();
    EXPECT_EQ(fsm.handle_event(Ev::Add), fsm::HandleEventResult::SUCCESS);
    EXPECT_EQ(fsm.handle_event(Ev::Unknown), fsm::HandleEventResult::UNHANDLED);
}
