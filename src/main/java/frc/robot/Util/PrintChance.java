package frc.robot.Util;

public class PrintChance {
  public static void printChance(String message, double chance) {
    if (Math.random() < chance) {
      System.out.println(message);
    }
  }

  public static void printChance(String message) {
    printChance(message, 0.02);
  }
}
