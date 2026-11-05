// SPDX-License-Identifier: Apache-2.0
#include "mpc_controller/control/TcpTrajectoryRecorder.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <gtest/gtest.h>

namespace zephyr::mpc {
namespace {
class TcpRecorderTest : public testing::Test {
protected:
    void SetUp() override {
        char pattern[] = "/tmp/mpc_recorder_test_XXXXXX";
        const char* directory = mkdtemp(pattern);
        ASSERT_NE(directory, nullptr);
        directory_ = directory;
    }
    void TearDown() override {
        if (!directory_.empty()) std::filesystem::remove_all(directory_);
    }
    std::vector<std::string> read(const char* name) const {
        std::ifstream file(directory_ / name);
        std::vector<std::string> lines;
        for (std::string line; std::getline(file, line);) lines.push_back(line);
        return lines;
    }
    std::filesystem::path directory_;
    const TcpTrajectorySample sample_{2.5, {1, 2, 3}, {0, 0, 0, 1}};
};

TEST_F(TcpRecorderTest, FlushesSeparateActualAndPredictedCsvOnStop) {
    TcpTrajectoryRecorder recorder;
    EXPECT_FALSE(recorder.enabled());
    recorder.appendActual(sample_); // Samples outside a session must not leak in.
    recorder.setEnabled(true, directory_.string());
    EXPECT_TRUE(recorder.enabled());
    recorder.appendActual(sample_);
    recorder.appendPredicted({3.5, {4, 5, 6}, {0, 0, 0, 1}});
    EXPECT_FALSE(std::filesystem::exists(directory_ / "real_tcp.csv"));
    recorder.setEnabled(false, directory_.string());
    EXPECT_FALSE(recorder.enabled());
    EXPECT_EQ(read("real_tcp.csv"), (std::vector<std::string>{
        "stamp_sec,x,y,z,qx,qy,qz,qw", "2.5,1,2,3,0,0,0,1"}));
    EXPECT_EQ(read("pred_tcp.csv"), (std::vector<std::string>{
        "stamp_sec,x,y,z,qx,qy,qz,qw", "3.5,4,5,6,0,0,0,1"}));
    recorder.appendActual(sample_);
    recorder.setEnabled(false, directory_.string());
    EXPECT_EQ(read("real_tcp.csv").size(), 2u);
}

TEST_F(TcpRecorderTest, RepeatedEnableKeepsSessionButNewSessionClearsBuffers) {
    TcpTrajectoryRecorder recorder;
    recorder.setEnabled(true, directory_.string());
    recorder.appendActual(sample_);
    recorder.setEnabled(true, directory_.string());
    recorder.appendActual(sample_);
    recorder.setEnabled(false, directory_.string());
    EXPECT_EQ(read("real_tcp.csv").size(), 3u);
    recorder.setEnabled(true, directory_.string());
    recorder.setEnabled(false, directory_.string());
    EXPECT_EQ(read("real_tcp.csv").size(), 1u);
    EXPECT_EQ(read("pred_tcp.csv").size(), 1u);
}

TEST_F(TcpRecorderTest, DestructorFlushesAnActiveSession) {
    {
        TcpTrajectoryRecorder recorder;
        recorder.setEnabled(true, directory_.string());
        recorder.appendActual(sample_);
    }
    EXPECT_EQ(read("real_tcp.csv").size(), 2u);
    EXPECT_EQ(read("pred_tcp.csv").size(), 1u);
}
TEST_F(TcpRecorderTest, BadDirectoryStopsRecordingWithoutThrowingAndAllowsNewSession) {
    const auto not_directory = directory_ / "file";
    std::ofstream(not_directory).put('x');
    TcpTrajectoryRecorder recorder;
    EXPECT_TRUE(recorder.setEnabled(true, (not_directory / "child").string()).empty());
    recorder.appendActual(sample_);
    EXPECT_FALSE(recorder.setEnabled(false, "").empty());
    EXPECT_FALSE(recorder.enabled());
    EXPECT_TRUE(recorder.setEnabled(true, directory_.string()).empty());
    recorder.appendActual(sample_);
    EXPECT_TRUE(recorder.setEnabled(false, "").empty());
    EXPECT_EQ(read("real_tcp.csv").size(), 2u);
}

TEST_F(TcpRecorderTest, FileOpenAndBufferedWriteFailuresAreReportedWithoutThrowing) {
    std::filesystem::create_directory(directory_ / "real_tcp.csv");
    TcpTrajectoryRecorder recorder;
    recorder.setEnabled(true, directory_.string());
    EXPECT_FALSE(recorder.setEnabled(false, "").empty());
    EXPECT_FALSE(recorder.enabled());
    std::filesystem::remove(directory_ / "real_tcp.csv");
    std::filesystem::create_symlink("/dev/full", directory_ / "real_tcp.csv");
    recorder.setEnabled(true, directory_.string());
    recorder.appendActual(sample_);
    EXPECT_FALSE(recorder.setEnabled(false, "").empty());
    EXPECT_FALSE(recorder.enabled());
}
} // namespace
} // namespace zephyr::mpc
