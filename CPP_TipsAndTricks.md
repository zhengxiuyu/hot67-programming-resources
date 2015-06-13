This page details a few tips and tricks you can employ to accomplish varying tasks and would not normally be explained in this tutorial. Some of them require a certain level of understanding. That level will be written under each heading.

Note that the levels listed here are the **absolute minimum** levels of knowledge you will need to implement them. It may be easier for you to learn them at a later time.



# Debugging and Fixing Errors #

**Required Level: Lesson 1**

When you start building programs, you may notice that your compiler may spit errors at you and not build your program, for example:

```
math1.cpp: In function ‘int main()’:
math1.cpp:9:12: error: ‘z’ was not declared in this scope
math1.cpp:11:1: error: expected ‘;’ before ‘return’
math1.cpp:11:8: error: expected ‘}’ at end of input
```

The errors that you get may look different depending on which compiler you use (what is shown above is from G++, which is what Code::Blocks uses). However, it is important that you know how to fix these errors. This section will show you how to deal with these errors.

Let's look at how to fix some of these errors.

## Errors ##

### Finding the error ###

Whenever you are given an error, whatever compiler you use should always tell you where the error is. It is important that you know how to determine where the error is. Below are two examples of how to do this with two compilers, G++ and Visual Studio

#### G++ ####
```
source_file.cpp:line:character: error...
```

Here, G++ will give you the name of the file that it found the error is in, the line number, the character from the left, and then it tells you the problem.

#### Visual Studio ####
```
\source_file.cpp(line): error...
```

Here, Visual Studio tells you the name of the file that it found the error in, the line number in parentheses, and then the problem.

### Common errors ###

_Note: The errors shown below are examples of errors given by G++. If you are using a different compiler, the language will look slightly different._

#### `'variable' was not declared in this scope` ####

You did not declare that variable in the current context (given by `In function:` or `In scope:`). There are usually three reasons for this error:

  * You forgot to declare that variable.
  * You misspelled that variable or its declaration.
  * You omitted a semicolon at the end of the declaration of the variable directly before this one.

#### `expected 'something' before 'key word'` ####

You omitted an important character. These are most often semicolons (`;`), but they can be other symbols.

Your compiler should also give you key words (`return`, a variable name, etc) as to where the error is in addition to the line and character number.

### Additional errors ###

There are lots of errors that you might encounter, and most of them depend highly on what you are programming. In general, if you have encountered an error you have not seen before, read it thoroughly. Most errors that your compiler gives you should be detailed enough to explain how to fix it. If it is too complex, then consult the Internet **with the words from the error that you think are important**.

## Debugging ##

Sometimes, your compiler will build your program, but your program will function improperly at runtime. This is usually due to improper code that the compiler was unable to find because the bug requires certain conditions within the program that might not always exist at runtime. In this case, there are a few techniques that you can apply to find the bad code and fix it.

In addition to what is given here, many IDEs (including Code::Blocks) have debugging tools that you can use. These are much more advanced and can help you find particularly elusive bugs.

### Verbose output ###

The easiest debugging method is to build it right into your program (this is why production-level software -- e.g. Android -- may contain development tools). You can do this by simply printing the variables that you think might be involved with the problem to the screen. Do this throughout the entirety of the program, and then build and run your program. Look closesly at the output, and see where it starts to look funky. Find that particular part of your code -- that is where the problem is.

### Condition testing ###

If the bug only seems to occur when given certain values and conditions, experiment with those variables/conditions (either by hard-coding them or have them input at runtime). Note which values and conditions are causing the bug, and use that to determine exactly what the program is doing to cause the problem.

# printf #
**Required Level: Lesson 1**

The `printf` function is another method of output from C (the language C++ is based on).  It allows you to print variable values right in a text string.

This will be useful later when you start programming the robot.

When you use it, you need to include the library `cstdio`.

```
#include <iostream>
#include <cstdio>

using namespace std;

int main()
{
    int x;
    x=5;
    printf("I have %d puppies!\n",x);
    
    return 0;
}
```

The code above prints "I have 5 puppies!"

Here is a shortened table of format specifiers (the `%d` above is an example) that `printf()` accepts:

_This table is from the www.cplusplus.com reference page on `printf()`_

| **Specifier** | **Type** | **Example** |
|:--------------|:---------|:------------|
| `d` or `i`    | Signed integer | `-49`       |
| `u`           | Unsigned integer | `435`       |
| `f`           | Floating point number | `58.9784`   |
| `c`           | Character | `g`         |
| `s`           | C String | `"hello"`   |

See [here](http://www.cplusplus.com/reference/cstdio/printf/) for a complete table and more details on `printf`.

Please note that printf will not accept C++ strings (variable type `string`) If you need to output a `string` in this manner, use this method for now:

| **Code** | **Output** |
|:---------|:-----------|
| `string mystring;` <br /> `mystring = "hello";` <br /> `printf("I would like to say %s", mystring.c_str());` | `I would like to say hello` |

Another way to do this would be to declare a [C string](CPP_Lesson6#C_Strings.md); however, you need to have learned the concepts in lesson 6 in order to do that.

# Random Numbers #
**Required Level: Lesson 1**

First off, the numbers you will obtain through this method are not quite random, as they are created through a predictable algorithm. However, for most purposes, this process is acceptable.

In order to use it, you must include the following libraries:

```
cstdlib
ctime
```

## The Random Number Generator ##

### Seeding ###
Such numbers are created through a random number generator. This generator must be seeded before it can generate any random numbers. The random number generator needs to be seeded ONCE and only ONCE. Seeding multiple times will render the random number generator useless.

Seed it like this:

```
srand(time(NULL));
```

This provides a random number seed that changes every time the program is run, because the seed is the time the `srand()` function is called.

You should generally always seed the random number generator at the beginning of `main`. Seeding it elsewhere (for example, in a class constructor or in another function) may actually seed it multiple times.

### rand ###

Then, you can generate random `int`s like this:

```
int my_int = rand();
```

Note that the `rand()` function takes no parameters. No matter what, the `rand()` function alone will generate a number from 0 to a value known as RAND\_MAX, which can vary, but is at least 32767 (the largest number an int is accepted to handle -- though it can handle more, but memory comes into play there).

Regardless, to bring it into a range, use the modulo operator, like so:

```
int my_int = rand() % 20;
```

This will generate a number from 0 to 19. If you want to generate numbers in a specific range, use the following formula:

```
rand() % ((max-min)+1) + min;
```

_Remember that addition and subtraction ignore the index number (the number you start from). This is why you need to add 1 after the range calculation._
_Also, **DO NOT TAKE THE RESULT OF THE ABOVE EQUATION AND WRITE `rand() % result`, AS THIS CHANGES THE RANGE OF NUMBERS TO BE GENERATED!** (Order of Operations - modulo is grouped with multiplication/division)_

For example:

```
int my_int = rand() % 31 + 1983;
```

will generate years from 1983 to 2013.

# I/O Manipulation #
**Required Level: Lesson 1**

If you are a stickler about presentation, there is a whole library for that.

```
#include <iomanip>
```

Note that the functions below only work on `cout`. For a full list of functions provided by this library, see [iomanip](http://www.cplusplus.com/reference/iomanip/). Some listed there will also work on `cin`.

Examples provided here are code fragments, meaning that they will not work on their own.

## Output operation ##
An **output operation** is one of the clauses you put between the `<<` operators in `cout` statements.

**Example**
```
cout<<OutputOperation1<<OutputOperation2<<OutputOperation3;
```

Note that the following functions are not considered output operations.

When considering how to set it up, the following functions do affect following `cout` statements, if applicable.

## setw ##
The `setw()` function allows you to **set the width** of a specific output operation.

```
iomanip::setw(int width)
```

### setw parameters ###
  * `width`: The number of characters that the following output operation should contain. If there is an excess, the [fill character](CPP_TipsAndTricks#setfill.md) is repeated and put before the characters output by the following operation.

### setw use ###
Place this function **before** the desired output operation.

**Code**
```
cout<<"Not affected"<<setw(30);
cout<<"Affected"<<" Not affected"<<endl;
```

_or_

```
cout<<"Not affected"<<setw(30)<<"Affected"<<" Not affected"<<endl;
```

**Output**
```
Not affected                      Affected Not affected
```

## setfill ##
The `setfill()` function allows you to set the fill character (normally a whitespace). The fill character is used if an output operation contains fewer characters than the [field width](CPP_TipsAndTricks#setw.md).

`setfill()` affects **all** output operations following it.

```
iomanip::setfill(char_type c)
```

### setfill parameters ###

  * `c`: Character to use as the fill character in the following output operation.

### setfill use ###
Place this function **before** the desired output operation.

| **Code** | **Output** |
|:---------|:-----------|
| `cout<<setfill('+')<<setw(10)<<"hi"<<endl;` | `++++++++hi` |

## Text alignment ##

These functions are actually a part of `iostream`, so `iomanip` is not required to use them.

They change the alignment of output operations depending on the [field width](CPP_TipsAndTricks#setw.md), not necessarily the screen size.

It is also important to note that these functions affect **all** output operations following their use, meaning if you want it to change back, you have to use the proper function.

Text is normally aligned to the right.

### left ###
Sets the output stream to put fill characters after successive output operations (causing a left alignment).

#### left parameters ####
The parameter for `left` is the following output operation, therefore, parentheses are unneccessary.

#### left use ####
Use `left` before the output operations to be left-aligned.

| **Code** | **Output** |
|:---------|:-----------|
| `cout<<setfill('+')<<setw(10)<<left<<"hi"<<endl;` | `hi++++++++` |

### right ###
Sets the output stream to put fill characters before successive output operations (causing a right alignment).

#### right parameters ####
The parameter for `right` is the following output operation, therefore, parentheses are unneccessary.

#### right use ####
Use `right` before the output operations to be right-aligned.

| **Code** | **Output** |
|:---------|:-----------|
| `cout<<setfill('+')<<setw(10)<<right<<"hi"<<endl;` | `++++++++hi` |

### internal ###
Sets the output stream to put fill characters in the middle of successive output operations (splitting the readout in two).

#### internal parameters ####
The parameter for `internal` is the following output operation, therefore, parentheses are unneccessary.

#### internal use ####
Use `internal` before the output operations to fill internally.

| **Code** | **Output** |
|:---------|:-----------|
| `cout<<setfill('+')<<setw(10)<<internal<<"hi"<<endl;` | `h++++++++i` |

# Clearing the Screen #
**Required Level: Lesson 1**

Clearing the screen actually depends on which platform you are using, because how the terminal is implemented between different platforms changes.

## Clearing the Screen on Windows ##

Use [this function](http://www.cplusplus.com/articles/4z18T05o/#Windows).

## Clearing the Screen on POSIX-Compliant Systems ##

By POSIX-compliant I basically mean any system derived from Unix -- the two most prominent examples of this are Mac OS X and Linux. This will work for any such system.

Unix-derived systems actually have a rather standard way to do this, since someone developed a library that made terminal handling easier. All you have to do is print the right control sequence to the terminal. The code below (borrowed from [here](http://www.cplusplus.com/articles/4z18T05o/#POSIX)) provides a fairly robust way to do it:

```
#include <unistd.h>
#include <term.h>

void clearScreen()
{
  if (!cur_term)
  {
    int result;
    setupterm(NULL, STDOUT_FILENO, &result);
    if (result <= 0)
      return;
  }
  
  putp(tigetstr("clear"));
}
```

The last line (the `putp(tigetstr("clear"))`) basically asks the terminal for the control sequence to clear the screen, and then prints it.

If you use this, you will need to link the right library (older Ubuntu systems use `-lterminfo`, newer Ubuntu systems use `-ltinfo`, other POSIX-compliant systems might use `-lcurses` or so on).

# Reading from Unbuffered Input #
**Required Level: Lesson 1**

Unbuffered input is basically input that has not been ended by a strike of the enter key. This is useful for writing programs that respond immediately to a key stroke.

Again, unbuffered input depends on the implementation of the terminal you are using, so this is platform-dependent.

## Unbuffered Input on Windows ##

Really, the easiest way to do this is using `conio.h`. There is a way to do it using the Windows API, but it requires knowledge from beyond this tutorial.

```
#include <conio.h>

char c;
c = _getch();

//or if _getch() doesn't work
c = getch();
```

## Unbuffered Input on POSIX-compliant systems ##

There are multiple different ways to do this. You could set the associated variable with `stty` or you could use the `getch()` function from `curses`. However, the former is a little tricky, and curses is not compatible with `stdin` and `stdout` (i.e. `cin` and `cout`), so you would have to learn all of `curses` to use it. I would suggest looking at [this](http://stackoverflow.com/a/912796).


# Input Validation #
**Required Level: Lesson 3**

When the user inputs data into your program, he/she may not know what data type you are looking for. If that is a concern, you can use the following method to keep your program running properly, should that happen.

```
#include <iostream>
#include <limits>
//you need this library!!

using namespace std;

int main () {

  unsigned myUInt;
  //note that this is the same as unsigned int myUInt;

  while (!cin>>myUInt)
  {
    cout<<"Please input a positive integer."<<endl;
    cin.clear();
    cin.ignore(numeric_limits<streamsize>.max(),'\n');
  }

  cout<<"You input a positive "<<myUInt<<"."<<endl;

  return 0;
}

```

The above code would run like this:

| **Input** | **Output** |
|:----------|:-----------|
| `2`       | `You input a positive 2.` |
| `abc`     | `Please input a positive integer.` |
| `-9`      | `Please input a positive integer.` |
| `0.25`    | `Please input a positive integer.` |


_Note that you may need to double the `cin.ignore()` statement_

## Why it works ##

Despite the fact that the `cin` statement is inside the parentheses for the `while` loop, the program still looks for input at that point. If the user inputs the wrong data type, `cin` throws a couple of flags (`fail` and/or `bad`, to be exact). The computer interprets this to be a `false` value, so it runs the loop.


_This resource was compiled with reference to http://www.cplusplus.com_