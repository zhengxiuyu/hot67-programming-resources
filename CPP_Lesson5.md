# Reading #

Read the following pages all the way through:

  * [Functions I](http://www.cplusplus.com/doc/tutorial/functions/)
  * [Functions II](http://www.cplusplus.com/doc/tutorial/functions2/)

# Extra Reading #

http://www.cprogramming.com/tutorial/lesson4.html

# Assignments #

Once you understand the content, complete these assignments.

## Temperature Converter ##

Write a function `celsius()` to convert degrees Fahrenheit to degrees Celsius (The conversion formula is `°C = 5/9 * (°F - 32)`). Use it to print a Fahrenheit-to-Centigrade table for -40 to 220 degrees Fahrenheit, in increments of 10 degrees.

Remember that `%f` is the `printf()` format to use for printing floating-point numbers. Also, it is important to note that the expression `5/9` is `0`, so you won't want to use integer division - divide `5.0` by `9.0` instead.

### Extra Practice ###
Do any of the following:

  * Allow the user to input a desired temperature to convert
    * You can also have them input a starting number and an ending number, and get a table from that.
      * **WARNING!** This is tricky!
  * Convert from Celsius to Fahrenheit

## Rectangle ##

Write a function `createrect(int x, int y)` that creates a rectangle out of x's.

i.e.

If you call `createrect(5,4)` you will receive:

```
xxxxx
x   x
x   x
xxxxx
```

HINT: you will need to use nested `for` loops like you did in the '[Right Triangle](WPI_Lesson3#Right_Triangle.md)' program.

### Extra Practice ###
Rewrite the program so that it can do any of the following:

  * Allow the user to change the size of the rectangle
  * Allow the user to create multiple rectangles in the same run (meaning the program cannot exit)
  * Allow the user to put a message in the rectangle **without** changing its dimensions or exceeding boundaries (pre-defined or user-defined dimensions, but the rectangle must stay a rectangle)
    * If the dimensions are too small for the message to fit, do not print anything. Instead, tell the user they cannot fit the message inside that rectangle, and have them try again.

| ← [Chapter 1: Basic C++ - Lesson 4: Switch/Case](CPP_Lesson4.md) | **Lesson 5** | [Lesson 6: Arrays](CPP_Lesson6.md) → |
|:-------------------------------------------------------------------|:-------------|:---------------------------------------|