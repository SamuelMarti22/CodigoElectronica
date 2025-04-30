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
 
## Prerequisites 

The Rule class is used to represent the production rules of a grammar. Each rule consists of a left-hand side (LHS) and a right-hand side (RHS), which are essential components in formal grammar representation. The Rule class is defined as follows:
```
class Rule:
    def __init__(self,simboloProduccion,producion):
        self.simboloProduccion = simboloProduccion
        self.produccion = producion
        self.first = None
```

Based on a .txt file, this code identifies the production rules, as well as the terminal and non-terminal symbols of a given grammar. It first reads the number of rules, then processes each one by checking its format and separating the production symbol from its derivation. It also sets the start symbol and stores each rule. Finally, it reads the input strings to be analyzed until it encounters a line with the character e, which marks the end of the input:

```
rules = []
strings = []
terminals = ['$']
nonTerminals = []

#Read the input file
with open("input.txt", "r") as file:
    #Read the first line of the file, which contains the number of rules
    num_rules = int(file.readline().strip())
    #Read each rule and store it in the rules list
    for i in range(num_rules):
        rule = file.readline().strip()
        parts = rule.split("->")
        #Confirm that the rule has the correct format (Production symbol -> Production).
        if len(parts) == 2:
            parts = [parte.strip() for parte in parts]
        else:
            print("Does not meet the requirements of the txt file")
            exit()
        #Separate the different productions of the same production symbol and store them in the rules list.
        parts[1] = parts[1].split(" ")
        if i == 0:
            #Define the start symbol of the grammar.
            startSymbolGrammar = parts[0]
        #Identify the terminal and non-terminal symbols and add them to their respective lists.
        for j in range(len(parts[1])):
            identify_terminal(parts[0])
            identify_terminal(parts[1][j])
            rule = Rule(parts[0], parts[1][j])
            rules.append(rule)
    #Read the strings to process and store them in the list of strings, until the character "e" is found.
    for line in file:
        line = line.strip()+"$"
        if line == "e$":
            break
        strings.append(line)
```
The identify_terminal function analyzes each production rule to categorize symbols as either terminal or non-terminal. It iterates through each character in the production: if the character is uppercase and hasn’t been added to the nonTerminals list, it is classified as a non-terminal. If the character is not uppercase (i.e., a terminal) and hasn’t been added to the terminals list, it is added as a terminal. This process ensures that symbols are uniquely categorized and stored, distinguishing between terminals and non-terminals in the grammar.

```
#Identify terminals and non terminals of grammar in each production.
def identify_terminal(production):
    for i in range(len(production)):
        #Take each character, evaluate if it is capital and has not been add in nonTerminals.
        if production[i].isupper() and production[i] not in nonTerminals: 
            nonTerminals.append(production[i])
        # If it not is capital then this character neither is non Terminal nor it is repet in terminals. Add this character in terminals.
        elif production[i] not in nonTerminals and production[i] not in terminals:
            terminals.append(production[i])
```         

To understand parsers, it's essential to work with the production rules of a grammar and the concepts of FIRST (the set of terminals that can appear at the beginning of a derivation from a non-terminal) and FOLLOW (the set of terminals that can appear immediately after a non-terminal in a production). These sets serve as the foundation for building the parsing tables used by different types of parsers.

```
diccFirst = {}
diccFollow = {}
for i in range(len(nonTerminals)):
    diccFirst[nonTerminals[i]] = []
    diccFollow[nonTerminals[i]] = []
    if nonTerminals[i] == startSymbolGrammar:
        diccFollow[nonTerminals[i]].append("$")

#Construction the first for every non terminal.
def first(production,diccFirst,flagLL):
    #Take the first character in the production of a rule.
    symbol = production.get_production()[0]
    #If the symbol is same the symbol of the production, then in the rule exist left recursion. Not is a grammar type LL(1).
    if symbol == production.get_production_symbol():
        print("Left Recursion")
        flagLL = False
        production.set_first(".")
    else:
        #If the symbol is a non-terminal, it adds it to the first of the production symbol unless it is repeated.
        if symbol in terminals:
            production.set_first(symbol)
            if symbol not in diccFirst[production.get_production_symbol()]:
                diccFirst[production.get_production_symbol()].append(symbol)
            else:
                flagLL = False
        else:
            #If the symbol is a non-terminal, it is necessary to add the symbols from the first of that non-terminal to the first of the production symbol.
            production.set_first(diccFirst[symbol])
            if symbol not in diccFirst[production.get_production_symbol()]:
                diccFirst[production.get_production_symbol()].extend(diccFirst[symbol]) 
                #Remove the duplicate symbols after adding the corresponding first symbols of the other symbol.
                diccFirst[production.get_production_symbol()] = list(set(diccFirst[production.get_production_symbol()])) 
    return flagLL
```
This function computes the First set for a given production and checks whether the grammar is valid for LL(1) parsing. It analyzes the first symbol of the production: if it's the same as the production's head, it detects left recursion and marks the grammar as not LL(1). If the symbol is a terminal, it adds it directly to the First set. If it's a non-terminal, it appends the First set of that symbol to the current one, ensuring no duplicates. Finally, it returns a flag indicating whether the grammar remains valid for LL(1) parsing.

```
#Construction of follow for each non-terminal.
def follow(diccFollow,nonTerminal):
    for i in range(len(rules)):
        #Find in each rule if it is part of it.
        if nonTerminal in rules[i].get_production():
            positionNonTerminal = rules[i].get_production().index(nonTerminal) #Toma la posicion donde se encuentra el non terminals.
            #If the symbol is at the end of the production
            if positionNonTerminal == len(rules[i].get_production())-1: 
                #If the rule only has the non-terminal symbol as production, add the follow symbols of this non-terminal.
                if rules[i].get_production_symbol() == nonTerminal:
                    diccFollow[nonTerminal].extend(diccFollow[rules[i].get_production_symbol()])
                #If the follow of that non-terminal symbol is defined and different from '$', add its value to the follow of the symbol in the rule where it is located.
                elif diccFollow[rules[i].get_production_symbol()] != [] and diccFollow[rules[i].get_production_symbol()] != ["$"]:
                    diccFollow[nonTerminal].extend(diccFollow[rules[i].get_production_symbol()])
                #If the production is different from the non-terminal symbol, it means it does not have a follow defined yet; recursion is done to define it and then add it
                elif rules[i].get_production_symbol() != nonTerminal:
                    diccFollow[nonTerminal].extend(follow(diccFollow, rules[i].get_production_symbol()))
                diccFollow[nonTerminal] = list(set(diccFollow[nonTerminal])) #Remove the duplicate characters after adding them.
            #If the non-terminal symbol is not at the end and is followed by a terminal, add this character to the follow list of the non-terminal symbol.
            elif rules[i].get_production()[positionNonTerminal+1] in terminals:
                diccFollow[nonTerminal].append(rules[i].get_production()[positionNonTerminal+1])
                diccFollow[nonTerminal] = list(set(diccFollow[nonTerminal]))  #Remove the duplicate characters after adding them.
            else:
                #If the symbol is not at the end and is followed by a non-terminal, then it should add the elements from the first of that non-terminal.
                firstNext = diccFirst[rules[i].get_production()[positionNonTerminal+1]].copy()
                #If the first includes the symbol e, then it should remove it.
                if "e" in firstNext:
                    firstNext.remove("e")
                    if diccFollow[rules[i].get_production()[positionNonTerminal+1]] != []:
                        #If there is a follow for this non-terminal, then it adds it.
                        firstNext.extend(diccFollow[rules[i].get_production()[positionNonTerminal+1]]) 
                    else:
                        #If there is no follow for this non-terminal, then it should perform recursion to find it and add it.
                        firstNext.extend(follow(diccFollow, rules[i].get_production()[positionNonTerminal+1])) 
                #Add the first to the follow of the non-terminal and remove the duplicate characters.
                diccFollow[nonTerminal].extend(firstNext)
                diccFollow[nonTerminal] = list(set(diccFollow[nonTerminal]))
    return diccFollow[nonTerminal] 
```
The follow function constructs the Follow set for a given non-terminal symbol in a grammar. It iterates over the production rules to check if the non-terminal appears in the right-hand side of any rule. If the non-terminal is found at the end of the production, it adds the Follow set of the non-terminal’s production symbol to its own Follow set. If the non-terminal is followed by a terminal symbol, that terminal is added to the Follow set. If followed by another non-terminal, it adds the First set of that non-terminal to the Follow set, handling special cases like epsilon (e) and recursing if necessary. Duplicate elements are removed to ensure unique entries. This process helps build the Follow set for each non-terminal symbol based on the grammar's rules.

## LL(1) Top-Down Parser 📝

### Explanation of the Parser 📖

The LL(1) parser is a **top-down** parsing method that reads input from **left to right**, constructing the parse tree from **top to bottom** using **one lookahead symbol** to make decisions.

$$
\begin{array}{c}
         S \\
       / | \ \\
      a\ B\ x \\
     |\\
     aa\ \ \ 
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

### Code for Developing It: Algorithm Flow for Building an LL(1) Parser💻

- Construction of the LL(1) Table, with Non-Terminals as Rows and Terminals as Columns
```
#Implementation of the function to construct the LL(1) table
def LLTableConstruction(diccFirst, diccFollow,rule):
    #If the first of the rule has the first character, create the dictionary with the tuple of the follows of the production symbol of the rule.
    if "e" in rule.get_first():
        for i in range(len(diccFollow[rule.get_production_symbol()])):
            parsingTableLL[(rule.get_production_symbol(), diccFollow[rule.get_production_symbol()][i])] = rule.get_production()
    else:
        #Create the dictionary with the tuple of the production symbol of the rule, its first, and the production.
        for i in range(len(rule.get_first())):
            parsingTableLL[(rule.get_production_symbol(), rule.get_first()[i])] = rule.get_production()
```

The LLTableConstruction function builds the LL(1) parsing table, where rows represent non-terminal symbols and columns represent terminal symbols. For each rule, if the First set of the production contains epsilon (e), the function uses the Follow set of the non-terminal to fill the table. Otherwise, it uses each symbol in the First set to map the rule's production into the table. This process ensures that the LL(1) table is correctly populated for predictive parsing.

- String Validation in LL(1) Parsing Using Stack-Based Processing
  
```
#Process of validating a string using SLR.
def processLL(string,flagProcessLL):
     # Acceptance case: If the stack contains the value "4" and the string is the end symbol '$', the process ends successfully.
    if stackLL[-1] == "$" and string == "$":
        #Empty the stack and add the process to the history.
        stackLL.pop(0) 
        createHistoriesLL(stackLL,string)
        flagProcessLL = True
        return flagProcessLL
    #If the last symbol of the stack matches the first symbol of the string, perform a match and remove the processed symbols. 
    if stackLL[-1] == string[0]:
        string = string[1:]
        stackLL.pop(-1)
        createHistoriesLL(stackLL,string)
        return processLL(string,flagProcessLL)  #Performs recursion to continue processing the string.
    else:
        #Try to get the new symbols for the stack, which are the result of matching the last symbol of the stack and the character from the string to process.
        try:
            addStack = parsingTableLL[stackLL[-1],string[0]]
        except KeyError:
            return flagProcessLL
        stackLL.pop(-1) #Remove the symbol taken by the matching.
        addStack=addStack[::-1] #Invert the result to be entered into the stack.
        stackLL.extend(addStack) #Add this variable to the stack.
        #If an epsilon is added to the queue, it should be removed, as it represents the empty string.
        if addStack[-1] == "e":
            stackLL.pop(-1)
            createHistoriesLL(stackLL,string)
            return processLL(string,flagProcessLL)
        else:
            createHistoriesLL(stackLL,string)
            return processLL(string,flagProcessLL) #Performs recursion to continue processing the string.

#Add to the variables the respective history obtained in processedLL.
def createHistoriesLL(stackSLRSymbol,inputSymbol):
    inputHistory.append(inputSymbol)
    stackSLRHistory.append(list(stackSLRSymbol)[::-1])
```
The LL(1) parser validates input strings using a stack and a parsing table. It compares the top of the stack with the current input symbol: if they match, both are consumed; otherwise, the parser looks up the appropriate production in the LL(1) table to expand the non-terminal. This process continues recursively until the string is accepted or rejected.

- Print the process of parsing a string"
```
#Function to print the syntax analysis process.
def print_parsing_process_LL(stackSLRHistory, inputHistory, resutlLL):
    #Crear DataFrame
    df = pd.DataFrame({    "Stack": [" ".join(stack) for stack in stackSLRHistory],
    "Input": [" ".join(cadena) for cadena in inputHistory]})
    #Replace empty cells with '-'.
    df.replace('', '-', inplace=True)

    if resutlLL:
        resultado = "Cadena aceptada"
    else:
        resultado = "Cadena rechazada"
    df.loc[len(df.index)] = [resultado, "-----"]

    print(df.to_markdown(tablefmt="grid"))
```
This function prints the syntax analysis process of an LL(1) parser. It creates a DataFrame using the history of the stack and input symbols, displaying them step by step. If the string is accepted, it shows "Cadena aceptada" (String accepted); if rejected, it shows "Cadena rechazada" (String rejected). Empty cells in the table are replaced with dashes ('-'). The result is printed in a markdown table format for better visualization.

---

## SLR(1) Bottom-Up Parser 🔽

### Explanation of the Parser 📚

The SLR (Simple LR) parser is a **bottom-up** parsing method that processes the input **left to right**, constructing the parse tree **from bottom to top**. It uses **one lookahead symbol** to decide which production rule to apply or whether to reduce a production. The SLR parser is more powerful than the LL(1) parser because it can handle a broader range of grammars, including some that are not suitable for LL(1) parsing, but it still relies on a single lookahead symbol to make parsing decisions.

$$
\begin{array}{c}
         S \\
       / | \ \\
      a\ B\ x \\
     |\\
     aa\ \ \ 
\end{array}
$$

In an SLR parser, the approach is bottom-up, where the parser reads the input left to right, but instead of constructing the parse tree from top to bottom, it reduces the input string to the start symbol, working from the terminals upwards.

In the given example, we start with the input string aa and attempt to reduce it step by step. The first step is to find the rightmost matching production in the grammar and apply it. If we have a rule like:

$$S \to  aBx \\newline$$
And the current input is aa, the parser starts by identifying that the substring a can be reduced according to the production rule for B, eventually reducing the string down to S. Each step in the reduction process involves finding the leftmost matching production to replace the current sequence of symbols.The process works from the bottom to the top, where each non-terminal is reduced based on the lookahead symbol until the start symbol is reached, which indicates the acceptance of the string. Thus, the flow is bottom-up, with the parsing happening from left to right while reducing and constructing the parse tree.

- **No ambiguities**: A production rule cannot have identical symbols in its **First** sets:

$$S \to  iaT | ieT\\newline$$
$$First(iaT)={i}\\newline$$
$$First(ieT)={i}\\newline$$

### Code for Developing It 🧑‍💻

Here you would add your code that implements the SLR(1) parsing algorithm.

