!/usr/bin/perl

use Getopt::Long;
use strict;
use warnings;

my $test = "";
my $regen = "";
my $clean = "";
my $diff = "";
my $stats = "";
my @pass ;

GetOptions (
            "test=s"    => \$test, 
            "regen"    => \$regen, 
            "clean"    => \$clean, 
            "diff"    => \$diff, 
            "stats"    => \$stats, 
#            "pass=s"    => \@pass, 
            ) 
 or die("Error in command line arguments\n");

my $make        = "make -f ~/Scripts/Makefile"; 

###  LLVM Args
my $llvm_bin  = "/team/cs526/sdasgup3/PRE/llvm-build/Release+Asserts/bin/";
my $load_pass  = "/team/cs526/sdasgup3/PRE/llvm-build/Release+Asserts/lib/libLLVMLCM.so"; 


my $clang       = "$llvm_bin/clang";
my $clangP       = "$llvm_bin/clang++";
my $llvmdis     = "$llvm_bin/llvm-dis";
my $llvmas      = "$llvm_bin/llvm-as";
my $llvmld      = "$llvm_bin/llvm-ld";
my $opt         = "$llvm_bin/opt";

if($clean ne "") {
  execute("rm -rf *.ll *.bc *.O");
  exit(0);
}

#my $opts = "-mem2reg -loop-rotate  -debug";
my $opts = "-mem2reg -loop-rotate -lcm -mem2reg -stats ";

if($regen ne "") {
  execute("rm -rf *.bc *.ll *.log");
}

my @splitarr = split(/[.]/, $test);
my $head = $splitarr[0];
my $ext = $splitarr[1];

if(defined($head)) {
  if($ext eq "c") {
    execute("$clang -O0 -emit-llvm -w -c $test -o $head.bc");
  } else {
    execute("$clangP -O0 -emit-llvm -w -c $test -o $head.bc");
  }

  execute("$llvmdis $head.bc -o $head.ll");

  execute("$opt -load $load_pass $opts  $head.bc -o  $head-lcm.bc ");
  execute("$llvmdis $head-lcm.bc -o $head-lcm.ll");
}

sub execute {
  my $args = shift @_;
  print "$args \n";
  system("$args");
}

exit
