//
// Created by arjunbadyal on 27/04/2022.
//
#pragma once
#include <memory>

#include <gflags/gflags.h>

#include <iostream>
#include "drake/common/drake_assert.h"

#include "lcmtypes/drake/lcmt_drake_signal.hpp"
//#include "lcm/lcm-cpp.hpp"
#include "drake/common/text_logging.h"

#include "drake/lcm/drake_lcm.h"

#include <atomic>
#include <chrono>
#include <mutex>
#include <stdexcept>
#include <thread>



    namespace drake {
        namespace lcm {
            namespace {

                using drake::lcmt_drake_signal;

                class MessageMailbox {
                public:
                    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MessageMailbox)

                    lcmt_drake_signal GetMessage() const {
                        lcmt_drake_signal result{};
                        std::lock_guard<std::mutex> lock(mutex_);
                        result = message_;
                        return result;
                    }

                protected:
                    MessageMailbox() = default;

                    void SetMessage(const lcmt_drake_signal& new_value) {
                        std::lock_guard<std::mutex> lock(mutex_);
                        message_ = new_value;
                    }

                private:
                    mutable std::mutex mutex_;
                    lcmt_drake_signal message_{};
                };

// Subscribes to LCM using DrakeLcm::Subscribe.
            class SubMailbox final : public MessageMailbox {
            public:
                //DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DutMailbox)

                SubMailbox(const std::string &channel, lcm::DrakeLcm *dut) {
                    auto subscription = dut->Subscribe(
                            channel, [this](const void *data, int size) {
                                this->Handle(data, size);
                            });

                    drake::log()->info(this->GetMessage().dim);

                    // By default, deleting the subscription should not unsubscribe.
                    subscription.reset();

                }

            private:
                void Handle(const void *data, int size) {
                    lcmt_drake_signal decoded{};
                    decoded.decode(data, 0, size);
                    SetMessage(decoded);
                    drake::log()->info(decoded.dim);
                    drake::log()->info(decoded.dim);


                }
            };
        }
    }
}