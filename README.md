[![DOI](https://zenodo.org/badge/18170897.svg)](https://zenodo.org/badge/latestdoi/18170897)

Partial Redundancy Elimination (PRE)
====================================

PRE is a compiler optimization that eliminates
expressions that are redundant on some but not necessarily all paths through a
program. We implement a PRE optimization pass in the LLVM framework, and evaluate the benefits 
on a variety of applications. PRE is a powerful technique that subsumes 
Common Subexpression Elimination (CSE) and Loop Invariant Code Motion (LICM), and hence has a potential to 
greatly improve performance.

Commands
-------

opts_lcm = "-mem2reg -loop-rotate -reassociate -lcm  -mem2reg -simplifycfy; 

$opt -load $load_pass  $opts_lcm   $test.bc -o  $test-lcm.bc
