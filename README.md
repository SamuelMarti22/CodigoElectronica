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

1. Clone the repository:
    ```bash
    git clone <repository-url>
    ```

2. Navigate to the project directory:
    ```bash
    cd <project-directory>
    ```

3. Install the required libraries:
    ```bash
    pip install -r requirements.txt
    ```

4. Run the script to start the analysis:
    ```bash
    python main.py
    ```

---

## LL(1) Top-Down Parser 📝

### Explanation of the Parser 📖

The LL(1) parser is a **top-down** parsing method that reads input from **left to right**, constructing the parse tree from **top to bottom** using **one lookahead symbol** to make decisions.

### Code for Developing It 💻

Here you would add your code that implements the LL(1) parsing algorithm.

---

## SLR(1) Bottom-Up Parser 🔽

### Explanation of the Parser 📚

The SLR(1) parser is a **bottom-up** parsing method that reads input from **left to right** and constructs the parse tree from **the leaves (bottom)** to the **root**. It uses one lookahead symbol and considers the **Follow** sets of the grammar for transitions.

### Code for Developing It 🧑‍💻

Here you would add your code that implements the SLR(1) parsing algorithm.

