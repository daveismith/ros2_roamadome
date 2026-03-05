#include <gtest/gtest.h>
#include "ros2_roamadome/roamadome_serial_port.hpp"

#include <cmath>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>

namespace ros2_roamadome
{

// Test observer implementation to track notifications
class MockObserver : public ISerialObserver
{
public:
  void onPositionUpdate(uint32_t degrees, double radians) override
  {
    positionUpdateCalled_ = true;
    lastDegrees_ = degrees;
    lastRadians_ = radians;
    positionUpdateCount_++;
  }

  void onVelocityUpdate(double rad_per_sec) override
  {
    velocityUpdateCalled_ = true;
    lastVelocity_ = rad_per_sec;
  }

  void onUnhandledLine(const std::string& line) override
  {
    unhandledLineCalled_ = true;
    lastUnhandledLine_ = line;
    unhandledLineCount_++;
  }

  bool positionUpdateCalled() const { return positionUpdateCalled_; }
  bool velocityUpdateCalled() const { return velocityUpdateCalled_; }
  bool unhandledLineCalled() const { return unhandledLineCalled_; }

  uint32_t lastDegrees() const { return lastDegrees_; }
  double lastRadians() const { return lastRadians_; }
  double lastVelocity() const { return lastVelocity_; }
  std::string lastUnhandledLine() const { return lastUnhandledLine_; }

  int positionUpdateCount() const { return positionUpdateCount_; }
  int unhandledLineCount() const { return unhandledLineCount_; }

  void reset()
  {
    positionUpdateCalled_ = false;
    velocityUpdateCalled_ = false;
    unhandledLineCalled_ = false;
    lastDegrees_ = 0;
    lastRadians_ = 0.0;
    lastVelocity_ = 0.0;
    lastUnhandledLine_ = "";
    positionUpdateCount_ = 0;
    unhandledLineCount_ = 0;
  }

private:
  bool positionUpdateCalled_ = false;
  bool velocityUpdateCalled_ = false;
  bool unhandledLineCalled_ = false;

  uint32_t lastDegrees_ = 0;
  double lastRadians_ = 0.0;
  double lastVelocity_ = 0.0;
  std::string lastUnhandledLine_;

  int positionUpdateCount_ = 0;
  int unhandledLineCount_ = 0;
};

// Helper class for managing temporary named pipes
class TemporaryPipe
{
public:
  explicit TemporaryPipe(const std::string& name = "/tmp/test_roamadome_")
    : pipePath_(name + std::to_string(getpid()) + "_" + std::to_string(random()))
  {
    // Create the named pipe
    if (mkfifo(pipePath_.c_str(), 0666) < 0) {
      if (errno != EEXIST) {
        std::cerr << "Failed to create FIFO: " << strerror(errno) << std::endl;
        pipePath_ = "";
      }
    }
  }

  ~TemporaryPipe()
  {
    if (!pipePath_.empty()) {
      unlink(pipePath_.c_str());
    }
  }

  const std::string& path() const { return pipePath_; }

  void writeData(const std::string& data)
  {
    // Open pipe for writing in non-blocking mode
    int fd = open(pipePath_.c_str(), O_WRONLY | O_NONBLOCK);
    if (fd >= 0) {
      ssize_t n = write(fd, data.c_str(), data.length());
      (void)n;  // Suppress unused variable warning
      close(fd);
    }
  }

private:
  std::string pipePath_;

  static unsigned int random()
  {
    static unsigned int seed = 0;
    return seed++;
  }
};

// Test fixture
class RoamadomeSerialPortTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a temporary pipe to simulate serial device
    pipe_ = std::make_unique<TemporaryPipe>();
    serialPort_ = std::make_unique<RoamadomeSerialPort>(pipe_->path());
  }

  void TearDown() override
  {
    if (serialPort_ && serialPort_->isOpen()) {
      serialPort_->close();
    }
    serialPort_.reset();
    pipe_.reset();
  }

  std::unique_ptr<TemporaryPipe> pipe_;
  std::unique_ptr<RoamadomeSerialPort> serialPort_;

  // Helper method to open the serial port
  // Note: we skip configurePort() for named pipes since they don't support termios
  void openPort()
  {
    ASSERT_TRUE(serialPort_->open()) << "Failed to open serial port at " << pipe_->path();
    // Skip configurePort for pipes - they don't support termios ioctls
    // In real hardware scenarios, configurePort would be called
  }

  // Helper to write data to pipe and allow time for read
  void writeDataAndWait(const std::string& data)
  {
    pipe_->writeData(data);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
};

// ============================================================================
// POSITIVE TESTS - Valid data successfully parsed
// ============================================================================

TEST_F(RoamadomeSerialPortTest, TestPositionCallback_SingleValidPosition)
{
  openPort();

  uint32_t capturedDegrees = 0;
  double capturedRadians = 0.0;
  bool callbackInvoked = false;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    callbackInvoked = true;
    capturedDegrees = deg;
    capturedRadians = rad;
  });

  writeDataAndWait("DOME POSITION: 123\n");
  serialPort_->read();

  EXPECT_TRUE(callbackInvoked) << "Callback was not invoked";
  EXPECT_EQ(capturedDegrees, 123);
  EXPECT_NEAR(capturedRadians, 123.0 * M_PI / 180.0, 0.001);
}

TEST_F(RoamadomeSerialPortTest, TestObserver_SingleValidPosition)
{
  openPort();

  MockObserver observer;
  serialPort_->registerObserver(&observer);

  writeDataAndWait("DOME POSITION: 180\n");
  serialPort_->read();

  EXPECT_TRUE(observer.positionUpdateCalled());
  EXPECT_EQ(observer.lastDegrees(), 180);
  EXPECT_NEAR(observer.lastRadians(), M_PI, 0.001);
}

TEST_F(RoamadomeSerialPortTest, TestPositionCallback_MultiplePositions)
{
  openPort();

  uint32_t lastDegrees = 0;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    lastDegrees = deg;
    (void)rad;
  });

  writeDataAndWait("DOME POSITION: 45\nDOME POSITION: 90\nDOME POSITION: 180\n");
  serialPort_->read();

  // Last position should win
  EXPECT_EQ(lastDegrees, 180);
}

TEST_F(RoamadomeSerialPortTest, TestPositionCallback_PositionRangeMin)
{
  openPort();

  uint32_t capturedDegrees = 999;
  double capturedRadians = 999.0;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    capturedDegrees = deg;
    capturedRadians = rad;
  });

  writeDataAndWait("DOME POSITION: 0\n");
  serialPort_->read();

  EXPECT_EQ(capturedDegrees, 0);
  EXPECT_NEAR(capturedRadians, 0.0, 0.001);
}

TEST_F(RoamadomeSerialPortTest, TestPositionCallback_PositionRangeMax)
{
  openPort();

  uint32_t capturedDegrees = 0;
  double capturedRadians = 0.0;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    capturedDegrees = deg;
    capturedRadians = rad;
  });

  writeDataAndWait("DOME POSITION: 359\n");
  serialPort_->read();

  EXPECT_EQ(capturedDegrees, 359);
  EXPECT_NEAR(capturedRadians, 359.0 * M_PI / 180.0, 0.001);
}

TEST_F(RoamadomeSerialPortTest, TestBothCallbackAndObserver_BothNotified)
{
  openPort();

  bool callbackInvoked = false;
  uint32_t callbackDegrees = 0;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    callbackInvoked = true;
    callbackDegrees = deg;
    (void)rad;
  });

  MockObserver observer;
  serialPort_->registerObserver(&observer);

  writeDataAndWait("DOME POSITION: 270\n");
  serialPort_->read();

  EXPECT_TRUE(callbackInvoked);
  EXPECT_EQ(callbackDegrees, 270);
  EXPECT_TRUE(observer.positionUpdateCalled());
  EXPECT_EQ(observer.lastDegrees(), 270);
}

TEST_F(RoamadomeSerialPortTest, TestPositionCallback_LineBufferingAcrossReads)
{
  openPort();

  uint32_t capturedDegrees = 0;
  bool callbackInvoked = false;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    callbackInvoked = true;
    capturedDegrees = deg;
    (void)rad;
  });

  // Write incomplete line
  writeDataAndWait("DOME POSIT");
  serialPort_->read();
  EXPECT_FALSE(callbackInvoked) << "Callback should not be invoked for incomplete line";

  // Write rest of line
  writeDataAndWait("ION: 45\n");
  serialPort_->read();
  EXPECT_TRUE(callbackInvoked) << "Callback should be invoked after line is complete";
  EXPECT_EQ(capturedDegrees, 45);
}

// ============================================================================
// NEGATIVE TESTS - Invalid data, no callback invoked
// ============================================================================

TEST_F(RoamadomeSerialPortTest, TestPositionCallback_NoUpdate_MalformedPosition)
{
  openPort();

  bool positionCallbackInvoked = false;
  bool unhandledCallbackInvoked = false;
  std::string unhandledLine;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    positionCallbackInvoked = true;
    (void)deg;
    (void)rad;
  });

  serialPort_->setUnhandledLineCallback([&](const std::string& line) {
    unhandledCallbackInvoked = true;
    unhandledLine = line;
  });

  writeDataAndWait("DOME POSITION: abc\n");
  serialPort_->read();

  EXPECT_FALSE(positionCallbackInvoked) << "Position callback should not be invoked for malformed data";
  EXPECT_TRUE(unhandledCallbackInvoked) << "Unhandled callback should be invoked";
  EXPECT_EQ(unhandledLine, "DOME POSITION: abc");
}

TEST_F(RoamadomeSerialPortTest, TestPositionCallback_NoUpdate_OutOfRange)
{
  openPort();

  bool positionCallbackInvoked = false;
  bool unhandledCallbackInvoked = false;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    positionCallbackInvoked = true;
    (void)deg;
    (void)rad;
  });

  serialPort_->setUnhandledLineCallback([&](const std::string& line) {
    unhandledCallbackInvoked = true;
    (void)line;
  });

  writeDataAndWait("DOME POSITION: 360\n");
  serialPort_->read();

  EXPECT_FALSE(positionCallbackInvoked) << "Position callback should not be invoked for out-of-range value";
  EXPECT_TRUE(unhandledCallbackInvoked) << "Unhandled callback should be invoked";
}

TEST_F(RoamadomeSerialPortTest, TestPositionCallback_NoUpdate_NegativeValue)
{
  openPort();

  bool positionCallbackInvoked = false;
  bool unhandledCallbackInvoked = false;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    positionCallbackInvoked = true;
    (void)deg;
    (void)rad;
  });

  serialPort_->setUnhandledLineCallback([&](const std::string& line) {
    unhandledCallbackInvoked = true;
    (void)line;
  });

  writeDataAndWait("DOME POSITION: -5\n");
  serialPort_->read();

  EXPECT_FALSE(positionCallbackInvoked);
  EXPECT_TRUE(unhandledCallbackInvoked);
}

TEST_F(RoamadomeSerialPortTest, TestPositionCallback_NoUpdate_NonPositionLine)
{
  openPort();

  bool positionCallbackInvoked = false;
  bool unhandledCallbackInvoked = false;
  std::string unhandledLine;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    positionCallbackInvoked = true;
    (void)deg;
    (void)rad;
  });

  serialPort_->setUnhandledLineCallback([&](const std::string& line) {
    unhandledCallbackInvoked = true;
    unhandledLine = line;
  });

  writeDataAndWait("#DPSTATUS OK\n");
  serialPort_->read();

  EXPECT_FALSE(positionCallbackInvoked);
  EXPECT_TRUE(unhandledCallbackInvoked);
  EXPECT_EQ(unhandledLine, "#DPSTATUS OK");
}

TEST_F(RoamadomeSerialPortTest, TestPositionCallback_NoUpdate_EmptyLine)
{
  openPort();

  bool positionCallbackInvoked = false;
  bool unhandledCallbackInvoked = false;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    positionCallbackInvoked = true;
    (void)deg;
    (void)rad;
  });

  serialPort_->setUnhandledLineCallback([&](const std::string& line) {
    unhandledCallbackInvoked = true;
    (void)line;
  });

  writeDataAndWait("DOME POSITION: \n");
  serialPort_->read();

  EXPECT_FALSE(positionCallbackInvoked);
  EXPECT_TRUE(unhandledCallbackInvoked);
}

TEST_F(RoamadomeSerialPortTest, TestPositionCallback_NoUpdate_PartialPositionLine)
{
  openPort();

  bool positionCallbackInvoked = false;
  bool unhandledCallbackInvoked = false;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    positionCallbackInvoked = true;
    (void)deg;
    (void)rad;
  });

  serialPort_->setUnhandledLineCallback([&](const std::string& line) {
    unhandledCallbackInvoked = true;
    (void)line;
  });

  writeDataAndWait("DOME POSITION:\n");
  serialPort_->read();

  EXPECT_FALSE(positionCallbackInvoked);
  EXPECT_TRUE(unhandledCallbackInvoked);
}

// ============================================================================
// REGISTRATION TESTS - Callback and Observer registration behavior
// ============================================================================

TEST_F(RoamadomeSerialPortTest, TestNoNotification_UnregisteredCallback)
{
  openPort();

  // No callback registered - should not crash
  writeDataAndWait("DOME POSITION: 100\n");
  EXPECT_NO_THROW(serialPort_->read());
}

TEST_F(RoamadomeSerialPortTest, TestNoNotification_UnregisteredObserver)
{
  openPort();

  // No observer registered - should not crash
  writeDataAndWait("DOME POSITION: 100\n");
  EXPECT_NO_THROW(serialPort_->read());
}

TEST_F(RoamadomeSerialPortTest, TestUnhandledLineCallback_NonPositionLines)
{
  openPort();

  std::string capturedLine;
  bool unhandledCallbackInvoked = false;

  serialPort_->setUnhandledLineCallback([&](const std::string& line) {
    unhandledCallbackInvoked = true;
    capturedLine = line;
  });

  writeDataAndWait("INFO: Device initialized\n");
  serialPort_->read();

  EXPECT_TRUE(unhandledCallbackInvoked);
  EXPECT_EQ(capturedLine, "INFO: Device initialized");
}

TEST_F(RoamadomeSerialPortTest, TestUnhandledLineCallback_ConsolidatedMultipleLines)
{
  openPort();

  int unhandledLineCount = 0;

  serialPort_->setUnhandledLineCallback([&](const std::string& line) {
    unhandledLineCount++;
    (void)line;
  });

  writeDataAndWait("#DPSTATUS OK\nINFO: Ready\n#DPREPORT50\n");
  serialPort_->read();

  EXPECT_EQ(unhandledLineCount, 3);
}

// ============================================================================
// BUFFERING TESTS - Line buffering and protocol handling
// ============================================================================

TEST_F(RoamadomeSerialPortTest, TestLineBuffering_MultipleCompleteLines)
{
  openPort();

  int positionUpdateCount = 0;
  uint32_t lastDegrees = 0;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    positionUpdateCount++;
    lastDegrees = deg;
    (void)rad;
  });

  writeDataAndWait("DOME POSITION: 30\nDOME POSITION: 60\nDOME POSITION: 90\n");
  serialPort_->read();

  EXPECT_EQ(positionUpdateCount, 3);
  EXPECT_EQ(lastDegrees, 90);
}

TEST_F(RoamadomeSerialPortTest, TestLineBuffering_CarriageReturnHandling)
{
  openPort();

  uint32_t capturedDegrees = 0;
  bool callbackInvoked = false;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    callbackInvoked = true;
    capturedDegrees = deg;
    (void)rad;
  });

  writeDataAndWait("DOME POSITION: 135\r\n");
  serialPort_->read();

  EXPECT_TRUE(callbackInvoked);
  EXPECT_EQ(capturedDegrees, 135);
}

TEST_F(RoamadomeSerialPortTest, TestLineBuffering_MixedValidAndInvalidLines)
{
  openPort();

  int positionUpdateCount = 0;
  int unhandledLineCount = 0;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    positionUpdateCount++;
    (void)deg;
    (void)rad;
  });

  serialPort_->setUnhandledLineCallback([&](const std::string& line) {
    unhandledLineCount++;
    (void)line;
  });

  writeDataAndWait("DOME POSITION: 45\n#DPSTATUS\nDOME POSITION: 90\n");
  serialPort_->read();

  EXPECT_EQ(positionUpdateCount, 2);
  EXPECT_EQ(unhandledLineCount, 1);
}

TEST_F(RoamadomeSerialPortTest, TestLineBuffering_PartialDataInMultipleCycles)
{
  openPort();

  int positionUpdateCount = 0;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    positionUpdateCount++;
    (void)deg;
    (void)rad;
  });

  // Send data in small chunks across multiple read cycles
  writeDataAndWait("DOM");
  serialPort_->read();
  EXPECT_EQ(positionUpdateCount, 0) << "No complete position yet";

  writeDataAndWait("E POSITION: 200");
  serialPort_->read();
  EXPECT_EQ(positionUpdateCount, 0) << "Still no newline";

  writeDataAndWait("\n");
  serialPort_->read();
  EXPECT_EQ(positionUpdateCount, 1) << "Position should be parsed after newline";
}

// ============================================================================
// EDGE CASES
// ============================================================================

TEST_F(RoamadomeSerialPortTest, TestWhitespaceHandling_LeadingWhitespace)
{
  openPort();

  uint32_t capturedDegrees = 0;
  bool callbackInvoked = false;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    callbackInvoked = true;
    capturedDegrees = deg;
    (void)rad;
  });

  writeDataAndWait("DOME POSITION:    150\n");
  serialPort_->read();

  EXPECT_TRUE(callbackInvoked);
  EXPECT_EQ(capturedDegrees, 150);
}

TEST_F(RoamadomeSerialPortTest, TestWhitespaceHandling_TrailingWhitespace)
{
  openPort();

  uint32_t capturedDegrees = 0;
  bool callbackInvoked = false;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    callbackInvoked = true;
    capturedDegrees = deg;
    (void)rad;
  });

  writeDataAndWait("DOME POSITION: 200  \n");
  serialPort_->read();

  EXPECT_TRUE(callbackInvoked);
  EXPECT_EQ(capturedDegrees, 200);
}

TEST_F(RoamadomeSerialPortTest, TestLargeNumberAfterPosition)
{
  openPort();

  bool positionCallbackInvoked = false;
  bool unhandledCallbackInvoked = false;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    positionCallbackInvoked = true;
    (void)deg;
    (void)rad;
  });

  serialPort_->setUnhandledLineCallback([&](const std::string& line) {
    unhandledCallbackInvoked = true;
    (void)line;
  });

  writeDataAndWait("DOME POSITION: 999999\n");
  serialPort_->read();

  EXPECT_FALSE(positionCallbackInvoked);
  EXPECT_TRUE(unhandledCallbackInvoked);
}

TEST_F(RoamadomeSerialPortTest, TestPositionPrefix_CouldMatchMultipleTimes)
{
  openPort();

  int positionUpdateCount = 0;

  serialPort_->setPositionCallback([&](uint32_t deg, double rad) {
    positionUpdateCount++;
    (void)deg;
    (void)rad;
  });

  // Line with "DOME POSITION:" appearing after other text
  writeDataAndWait("INFO: Last DOME POSITION: 75\n");
  serialPort_->read();

  EXPECT_EQ(positionUpdateCount, 1) << "Should find DOME POSITION: even if later in line";
}

}  // namespace ros2_roamadome

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
