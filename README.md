my $opts_lcm = "-mem2reg -loop-rotate   -reassociate -lcm  -mem2reg -simplifycfg   ";
$opt -load $load_pass  $opts_lcm   $test.bc -o  $test-lcm.bc
