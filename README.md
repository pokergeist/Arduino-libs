# Arduino Plot(_u1) lib

The Plot class helps you generate and accumulate variable strings to be sent to the Arduino IDE's Serial Plotter.

After instantiation incrementally add() a label, value and an optional offset. The value is appended to the label by default. The value will be plotted at value + offset. You can specify min and max graph constraints with constrain_on(min, max) and disable them with constrain_off().

The class will append subsequently added value strings with a comma, and terminate the string with "\n" when print(Serial) is called.

The directory and .h files have been renamed with the _u1 suffix to hopefully avoid a name collision in the future. u1 indiates a simple 1st level utility.

Usage:

    #include <Plot_u1.h>
    
    Plot plot; // instanticate
    
    void setup(void) {
      Serial.begin(9600);
      delay(100);
    }
    
    void loop(void) {
      .... manipulate x ...
      plot.add("sin_x", sin(x), 2);   // plot values offset at +/-2
      plot.add("cos_x", cos(x), -2);
      plot.print(Serial); // see ~ "sin_x=0.50:2.50,cos_x=0.87:-1.13\n"
      ....
    }

If you use pointers to character strings and floats, and dereference them in the add call you can dynamically create some interesting graph sets. e.g.,

    char* label1;
    float *value1_p;
    float offset1;
    .... set the graph values somewhere
    label1 = "vel";
    value1_p = &velocity;
    offset1 = 0;
    ... loop ...
      velocity = ...
      ...
      plot.add(label1, *value1_p, offset1);

Enjoy.
