# Final Project - Syntactic Analyzer Type LL(1) AND SLR(1) 📖

In this assignment, the step-by-step process of analyzing a set of strings using two types of syntactic analyzers will be demonstrated: LL(1) (top-down parser) and SLR(1) (bottom-up parser). The process includes everything from identifying the type of grammar, constructing the parsing tables and automaton, to validating the strings.

## Contents 🤔

- [Team 👥](#team)
- [Development Environment 🖥️](#development-environment)
- [Instructions for Running ▶️](#instructions-for-running)
- [LL(1) Top-Down Parser 📝](#ll1-top-down-parser)
    - [Explanation of the Parser 📖](#explanation-of-the-parser)
    - [Code for Developing It 💻](#code-for-developing-it)
- [SLR(1) Bottom-Up Parser 🔽](#slr1-bottom-up-parser)
    - [Explanation of the Parser 📚](#explanation-of-the-parser-1)
    - [Code for Developing It 🧑‍💻](#code-for-developing-it-1)

---

## Team 👥

- **Team Members**: [List your team members here]

## Development Environment 🖥️

- **Operative System:** Windows 11  
- **Programming language:** Python 3.12  
- **Tools:** Visual Studio Code, Graphviz
- **Required Libraries**: Pandas, Graphviz

## Instructions for Running ▶️

## Instructions for running ▶️🏃‍♂️

- 1. Install pandas. To do this, type the following command in the terminal:
```
pip install pandas
```
- 2. Install Graphviz. To do this, type the following command in the terminal:
```
sudo apt-get update
sudo apt-get install graphviz
pip install graphviz
```
- 3. Run the main file

## LL(1) Top-Down Parser 📝

### Explanation of the Parser 📖

The LL(1) parser is a **top-down** parsing method that reads input from **left to right**, constructing the parse tree from **top to bottom** using **one lookahead symbol** to make decisions.

$$
\begin{array}{c}
         S \\
       / | \ \\
      A\ B\ x \\
/    |\\
 aa\ \ \ aa\ \ 
\end{array}
$$

In this example, we start from the initial symbol **S**, which produces **A B X**. The parser will take the leftmost non-terminal (**A**) and perform the possible syntactic derivation until it reaches a terminal symbol, as seen with `aa`. Then, it will continue with **B** and repeat the process, until all non-terminals have been derived. As we can observe, the parsing flow goes from top to bottom and from left to right (**Top-Down, Left-to-Right**).

To build an **LL(1)** parser (i.e., **Top-Down** with **1 lookahead**), the following rules must be satisfied:

- **No ambiguities**: A production rule cannot have identical symbols in its **First** sets:

$$S \to  iaT | ieT\\newline$$
$$First(iaT)={i}\\newline$$
$$First(ieT)={i}\\newline$$
  

- **No left recursion**: Left recursion must be avoided, as it creates an infinite loop when computing the **First** and **Follow** sets:

$$S \to  Sa \\newline$$

- **No non-determinism**: Non-deterministic rules are not allowed, because the parser can only use **one lookahead symbol** to decide:

$$S \to  aA |aB \\newline$$
$$A \to  d \\newline$$
$$B \to  c \\newline$$

In this case, both productions of **S** start with the terminal `a`, so the parser cannot decide deterministically.

### Code for Developing It 💻

Here you would add your code that implements the LL(1) parsing algorithm.

---

## SLR(1) Bottom-Up Parser 🔽

### Explanation of the Parser 📚

The SLR(1) parser is a **bottom-up** parsing method that reads input from **left to right** and constructs the parse tree from **the leaves (bottom)** to the **root**. It uses one lookahead symbol and considers the **Follow** sets of the grammar for transitions.

### Code for Developing It 🧑‍💻

Here you would add your code that implements the SLR(1) parsing algorithm.

