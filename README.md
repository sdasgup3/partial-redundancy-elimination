Partial Redundancy Elimination (PRE) is a compiler optimization that eliminates
expressions that are redundant on some but not necessarily all paths through a
program. In this project, we implemented a PRE optimization pass in LLVM 
and measured results on a variety of applications. It's a powerful technique that subsumes 
Common Subexpression Elimination (CSE) and Loop Invariant Code Motion (LICM), and hence has a potential to 
greatly improve performance.

opts_lcm = "-mem2reg -loop-rotate   -reassociate -lcm  -mem2reg -simplifycfy;

$opt -load $load_pass  $opts_lcm   $test.bc -o  $test-lcm.bc
