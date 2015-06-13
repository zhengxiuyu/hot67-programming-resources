# Reading #

Go here:

[Arrays](http://www.cplusplus.com/doc/tutorial/arrays/)

and read through the page.

# Extra Reading #

http://www.cprogramming.com/tutorial/lesson8.html

# C Strings #

You are likely aware of this, but C++ is actually derived from another language called C. In C, there is no `string` data type. Instead, C programmers use an array of `chars`.

Since many functions are actually C functions ported to C++ (like `printf`), they still rely on C strings. If you need to use a string in a C function, you can do one of two things:

  * Declare an array of `char`s
  * Use the `c_str()` function of `string` that converts a C++ `string` to a C string temporarily
    * i.e. `mystring.c_str();`

# Assignments #

Once you understand the content, complete these assignments.

## Pancake Glutton ##

Write a program that asks the user to enter the number of pancakes eaten for breakfast by 10 different people (Person 1, Person 2, ..., Person 10)
Once the data has been entered the program so that it outputs a list in order of number of pancakes eaten of all 10 people.

**Example:**
```
Person 4: ate 10 pancakes
Person 3: ate 7 pancakes
Person 8: ate 4 pancakes
...
Person 5: ate 0 pancakes
```

### Extra Practice ###

Sort the people in the other direction.

## Dungeon Crawl ##

Make a program that outputs a simple 10 x 10 grid based gameboard to the screen using characters.

**Example:**
```
. . . . . E . . . .
. G . . . . . . . .
. . . . . . T . . .
. . . . . . . . . .
. . . . T . . . . .
. . . . . . T . . .
. . . . . . . . . X
. . . . . . . T . .
. . . E . . . . . .
. T . . . . . . . .
```


Allow the user (marked by G) to move either up, down, left, or right each turn. If the player steps on a trap (marked by T) then they lose. If they make it to the treasure 'X' then they win.

Add enemies (denoted by E) that move [randomly](CPP_TipsAndTricks#Random_Numbers.md) in any direction once per turn (enemies, just like traps, cause the player to lose if touched).

If you don't want to have the user hit enter (for example, you want them to use the WASD keys to navigate), look at [unbuffered input](CPP_TipsAndTricks#Reading_from_Unbuffered_Input.md).

HINT: Don't let the player move off the gameboard! You program will crash if they move off the top or bottom of the board!
(the same holds true for enemies)

### Extra Practice ###
Do any of the following:

  * Create multiple difficulty levels
  * Create a dynamic map (the map changes between runs)
    * HINT: [rand()](CPP_TipsAndTricks#Random_Numbers.md)

| ← [Lesson 5: Functions](CPP_Lesson5.md) | **Lesson 6** | [Lesson 7: Data Structures](CPP_Lesson7.md) → |
|:------------------------------------------|:-------------|:------------------------------------------------|