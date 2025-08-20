package com.team581.util;

import com.team581.GlobalConfig;
import dev.doglog.DogLog;
import java.util.function.BooleanSupplier;

public class FeatureFlag {
  public static BooleanSupplier of(String name, boolean defaultValue) {
    if (GlobalConfig.IS_DEVELOPMENT) {
      return DogLog.tunable("FeatureFlags/" + name, defaultValue);
    }

    return () -> defaultValue;
  }

  private FeatureFlag() {}
}
