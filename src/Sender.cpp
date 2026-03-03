#include <mpc-rbt-solution/Sender.hpp>

void Sender::Node::run()
{
  while (errno != EINTR) {
    if ((std::chrono::steady_clock::now() - timer_tick) < timer_period) continue;
    timer_tick = std::chrono::steady_clock::now();

    callback();

    //std::this_thread::sleep_for(std::chrono::seconds(2));
  }
}

void Sender::Node::onDataTimerTick()
{
  data.timestamp =
    static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());

  data.x += 1.5;
  data.y += 2.0;
  data.z += 0.5;

  Socket::IPFrame frame{
    .port = config.remotePort,
    .address = config.remoteAddress,
  };
  RCLCPP_INFO(logger, "Sending data to host: '%s:%d'", frame.address.c_str(), frame.port);

  RCLCPP_INFO(logger, "\n\tstamp: %ld", data.timestamp);

  Utils::Message::serialize(frame, data);

  this->send(frame);
}
