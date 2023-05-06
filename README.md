# mahjongAsst
Japanese Mahjong Legacy Stick Scorer

  A Library for legacy scoring system of Japanese mahjong tables.
  Legacy mahjong scorers implement special score sticks containing electrical elements.
  My enquiries clarified that the elements are R / L / C passive components.
  This library measures parallel resistances/capacitances of stack-piled score sticks,
  and convert the values into actual scores of 4 mahjong players
  
# Features
-Realtime score tracking for all 4 people

-Outputs error message when the sticks are in wrong places

-Manage display mode(NORMAL, DIFF, PM) with push buttons for each player

-Manage honbas with a push button
  
# Supported Models
  
  # Resistors
  [AMOS MONSTER, NINJA, etc](doc/AMOS_MONSTER.txt)
  
  # Capacitors
  [CENTURY TENPAL GOLD](doc/CENTURY_TENPAL_GOLD.txt)
  
  [CENTURY TENPAL SILVER](doc/CENTURY_TENPAL_SILVER.txt)
  # Custom
  You make your own system, becuase everything is just a passive element value.
  
# Example Diagram
[Example](doc/PICO_AMOS_MONSTER.pdf)


MCU : Raspberry Pi Pico(RP2040)

Model : AMOS MONSTER (in fact resistors equivalent to the model)
