// Copyright 2023 Áron Svastits
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "decode_test.h"  // NOLINT

#include "kuka_rsi_hw_interface/rsi_command_handler.hpp"

TEST_F(DecodeTest, DecodeSuccessful) {
  kuka_rsi_hw_interface::RSICommandHandler rsi_command_handler;

  char input_string[] =
    "<Rob TYPE=\"KUKA\"><RIst X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\"" \
    " /><RSol X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\" /><AIPos A1=\"0.0\"" \
    " A2=\"-90.0\" A3=\"90.0\" A4=\"0.0\" A5=\"90.0\" A6=\"0.0\" /><ASPos A1=\"0.0\" A2=\"-90.0\"" \
    " A3=\"90.0\" A4=\"0.0\" A5=\"90.0\" A6=\"0.0\" /><Delay D=\"0\" /><IPOC>0</IPOC></Rob>";
  ASSERT_TRUE(rsi_command_handler.Decode(input_string, 1024));
}
