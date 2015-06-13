<font color='red' size='3'>This is one of the most important lessons, please make sure you have a good understanding of this section.  It DIRECTLY applies to how we program the robot.</font>

# Reading #

Read the following pages **thoroughly**:

  * [Classes I](http://www.cplusplus.com/doc/tutorial/classes/)
  * [Classes II](http://www.cplusplus.com/doc/tutorial/classes2/)

# Extra Reading #

http://www.cprogramming.com/tutorial/lesson12.html

# Assignment #

Once you understand the content, complete this assignment.

## WAR ##

### The Backstory ###

WAR has swept through Middle Earth like a tornado. Of course, the actual physical WAR with elves shooting arrows and orcs dropping like flies has come to Middle Earth. But also the card game WAR has become very popular throughout the land, particularly in local taverns among the young hip hobbit crowd. But with so many hobbits out of town on life-or-death quests to destroy magical rings, it's hard to find a partner to play cards with. If only there was a way to play single-player WAR, perhaps with some sort of magical lightbox. Who among you is lucky enough to rise to the challenge?

### Your Assignment ###

Your goal is to write a simplified single-player version of the card game WAR, also know as In-Between. The player will start with $100. At each round of play, the player will be shown two cards and decides how much to bet that the third card's rank is in between. Note the player is not allowed to bet more money than they currently have. The player should be told whether they won or lost and their total amount of money is updated accordingly. The game ends when the player is down to $0 or their amount exceeds $1,000.

A sample run is shown below. Note that betting $0 is equivalent to skipping a turn. Also note the user is prompted to re-enter their bet amount if they try to bet more than they actually have. When checking if the third card is "in between", the order in which the first two cards are presented should not matter. For example, a "Jack" is in between "Five and Queen" and also in between "Queen and Five". If the third card has the same rank as one of the original two cards, you may treat that as a loss.

Do NOT allow the player to bet negative money (that's cheating)! _HINT: We're talking about **unsigned** money_


> `You have $100.
> You got a Eight of Diamonds and a Seven of Clubs.
> How much do you want to bet the next card is in between? 200
> You only have $100 to bet! Enter a new bet: 0
> You got a Eight of Hearts.
> TOO BAD! You lose $0.

> You have $100.
> You got a Queen of Hearts and a Five of Spades.
> How much do you want to bet the next card is in between? 100
> You got a Jack of Clubs.
> COOL! You win $100.

> You have $200.
> You got a Two of Diamonds and a King of Clubs.
> How much do you want to bet the next card is in between? 200
> You got a Ace of Spades.
> TOO BAD! You lose $200.

> You are out of money. GAME OVER.
> Press any key to continue.`

### Programming Requirements ###

You must use [this cards.h.](https://hot67-programming-resources.googlecode.com/svn/trunk/tutorial/cards.h) **Do not make any changes to this header.**

You are expected to use the two classes Player and Card which are declared in the header file cards.h. You should define all class member functions in the source file cards.cpp and the main routine in the source file main.cpp. You should be able to figure out what the member functions do by looking at cards.h.

#### The Player Class ####
You should track the player's money using the Player class. You should define the member functions as given in the cards.h file.

#### Playing Cards ####
Every playing card has a suit and a rank. The possible suits, in no particular order, are:

  * Hearts
  * Diamonds
  * Clubs
  * Spades

The possible ranks, in order from lowest to highest, are:

  1. Ace
  1. Two
  1. Three
  1. Four
  1. Five
  1. Six
  1. Seven
  1. Eight
  1. Nine
  1. Ten
  1. Jack
  1. Queen
  1. King

Assume that each card has a value of what is given in the list (it is also fine to use numbers 0-12 instead of 1-13).

##### The Card Class #####
The class Card should have the "less than" operator defined that compares the ranks. For example, if `card1` is the Eight of Clubs and `card2` is the Queen of Hearts, then

```
card1 < card2
```

is a true statement. The suit has no effect on how the cards are compared. Your operator should make use of the public function `rank2num()`, which you may want to define first.

The default constructor for the Card class should assign a random suit and rank to the card. This allows us to "draw" a new card every time we call the constructor.

In order to generate random numbers, use the [rand()](CPP_TipsAndTricks#Random_Numbers.md) function. The `cstdlib` and `ctime` libraries have already been included for you in cards.h, so you don't have to.

For example, you could set the suit like this:
```
        int rand_num = 1 + rand() % 4;  //Random # between 1 and 4.
	switch(rand_num) {
		case 1: suit = "Hearts"; break;
		case 2: suit = "Diamonds"; break;
		case 3: suit = "Clubs"; break;
		case 4: suit = "Spades"; break;
	}
```

### Extra Practice ###
Do any of the following:

  * Make the cards dynamic memory (pointers), if you haven't already
  * Add multiplayer capability (still leave single player)
    * If you do this, put single player game and multiplayer game in separate functions.
  * Have the user set the starting money
    * Do not allow them to start with $1000
    * For multiplayer: All players must start with the same money

| ← [Lesson 8: Pointers](CPP_Lesson8.md) | **Lesson 9** | [WPI Tutorial](WPI_Lesson1.md) → |
|:-----------------------------------------|:-------------|:-----------------------------------|