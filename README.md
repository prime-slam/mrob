# skmr
The Soltech Mobile Robotics library includes a core set of functionalities required for a unified framework in robotics. In addition, some of the research topics of our lab and their implemented algorithms is present in the current library, being a self-contained library for almost any robotics topic.

## Dependencies
Eigen

## Coding conventions
Coding conventions are necessary to maintain homogeneity and readability across all the project. Here are some conventions that we _should_ follow:

* BSD/Allman conventions: -like, ie. brace on the next line from a control statement, indented on the same level. In switch-case statements the cases are on the same indent level as the switch statement.
* Indents use 4 spaces instead of tabs. Tabs are not used.
* Class and struct names camel-case and beginning with an uppercase letter.
* Variables are in lower camel-case. Member variables have an underscore appended. For example `odometryObs`, `localNodes_`.
* Constants and enumerations are in uppercase. For example M_PI.
* Class definitions proceed in the following order:

  - public constructors and the destructor
  - public virtual functions
  - public non-virtual member functions
  - public static functions
  - public member variables
  - public static variables
  - repeat all of the above in order for protected definitions, and finally private
* Header files are commented using one-line comments beginning with / &ast &ast to mark them, comments are important.

