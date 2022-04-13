# Arduino-Plot-lib

The Plot class helps you manage strings sent to the Arduino IDE Serial Plotter.

After instantiation incrementally add() a label, value and an optional offset. The value is appended to the label by default. The value will be plotted at value + offset. You can specify min and max graph constraints with constrain_on(min, max) and disable them with constrain_off().

The class will append subsequently added value strings with a comma, and terminate the string with "\n" when print(Serial) is called.
