# Reading #

Go here:

[Loops](http://www.cplusplus.com/doc/tutorial/control/#loops)

and read through that section.

# Extra Reading #

http://www.cprogramming.com/tutorial/lesson3.html

# Assignments #

Once you understand the content, complete these assignments.

## Tip: `printf` ##

You are strongly encouraged to give the [printf()](CPP_TipsAndTricks#printf.md) function some consideration when completing the assignments.

## Counter ##
Create a **For loop** that counts 1 to 101.  Count up by 4.  Display the number each time it goes through the loop.

## Counterintuitive Exit ##

Create a **do-while loop** that asks the user if they would like to quit the program. Keep on asking the question until the user says no.  If the user does not enter a proper response (yes or no), tell the user they entered an improper answer and ask if they would like to quit again.

## Right Triangle ##

Write a program to print this triangle:
```
        +
        ++
        +++
        ++++
        +++++
        ++++++
        +++++++
        ++++++++
        +++++++++
        ++++++++++
```
Don't use ten printf statements; use **two nested loops** (a loop in a loop) instead. You'll have to use braces around the body of the outer loop if it contains multiple statements:
```
        for(i = 1; i <= 10; i = i + 1)
        {
                /* multiple statements */
                /* can go in here */
        }
```
(Hint: a string you hand to `printf` or `cout` does not have to contain the newline character `\n`, or in the case of `cout`, the newline function `endl`.)

### Extra Practice ###
Rewrite this program to allow the user to pick the height of the triangle, still following the same line pattern. You may want to use an `unsigned int` in combination with an [input check](CPP_TipsAndTricks#Input_Validation.md) to prevent the user from creating impossible triangles.

| ← [Lesson 2: If/Else Statements](CPP_Lesson2.md) | **Lesson 3** | [Lesson 4: Switch/Case](CPP_Lesson4.md) → |
|:---------------------------------------------------|:-------------|:--------------------------------------------|