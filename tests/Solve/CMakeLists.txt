cmake_minimum_required (VERSION 3.16)

add_test_with_conf(SimulatorSelfTest Solve/simulator_self.conf)
add_test_with_conf(Static5mTest Solve/static5m.conf)
add_test_with_conf(Spin3mTest Solve/spin3m.conf)
add_test_with_conf(Spin5mTest Solve/spin5m.conf)
add_test_with_conf(LargeCircle5mTest Solve/large_circle5m.conf)
add_test_with_conf(Sentry8mTest Solve/sentry_8m.conf)
add_test_with_conf(Sentry5mTest Solve/sentry_5m.conf)
add_test_with_conf(Sentry3mTest Solve/sentry_3m.conf)
add_test_with_conf(MovingStaticTest Solve/moving_static5m.conf)
add_test_with_conf(StaticMovingTest Solve/static_moving5m.conf)
add_test_with_conf(DualMovingTest Solve/dual_moving5m.conf)
add_test_with_conf(ArmorLocator3mTest Solve/armor_locator_3m.conf)
add_test_with_conf(ArmorLocator10mTest Solve/armor_locator_10m.conf)
add_test_with_conf(ArmorLocatorFullTest Solve/armor_locator_full.conf)
add_test_with_conf(ArmorLocatorDistTest Solve/armor_locator_dist.conf)
