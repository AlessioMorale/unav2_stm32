#include "CppUTest/TestHarness.h"
#include "configuration.h"
#include <vector>

TEST_GROUP(ConfigurationChecks){
    void setup(){}    //
    void teardown(){} //
};

class configurationObserverTester : public unav::ConfigurationObserver {
  public:
    void configurationUpdated(const unav::ConfigurationMessageTypes_t configuredItem){
      (void)configuredItem;
      received.push_back(configuredItem);
    }
    std::vector<unav::ConfigurationMessageTypes_t> received;
};

TEST(ConfigurationChecks, CanCheckConfigurations) {

  configurationObserverTester tester;
  unav::Configuration configuration;
  configuration.Attach(&tester);

  unav::configuration_message_t message;
  message.header.type = unav::message_types_t::inbound_BridgeConfig;
  message.bridgeconfig.pwm_frequency = 1000;
  (void)message;
  configuration.set(&message);

  CHECK(tester.received.front() ==  unav::ConfigurationMessageTypes_t::bridgeconfig);
  CHECK(configuration.getBridgeConfig().pwm_frequency == 1000);
  tester.received.pop_back();

  message.header.type = unav::message_types_t::inbound_OperationConfig;
  message.operationconfig.reset_to_dfu = true;
  configuration.set(&message);
  CHECK(tester.received.front() ==  unav::ConfigurationMessageTypes_t::operationconfig);
  CHECK(configuration.getOperationConfig().reset_to_dfu == true);
  tester.received.pop_back();

  message.operationconfig.reset_to_dfu = false;
  configuration.set(&message);
  CHECK(tester.received.front() ==  unav::ConfigurationMessageTypes_t::operationconfig);
  CHECK(configuration.getOperationConfig().reset_to_dfu == false);
  tester.received.pop_back();
}
