//===--------------  LCM.cpp - Lazy Code Motion --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file was developed by the LLVM research group and is distributed under
// the University of Illinois Open Source License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//  
//      TODO : Add a description here 
//===----------------------------------------------------------------------===//

#include "llvm/Transforms/Scalar.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/Pass.h"
#include "llvm/IR/Constants.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/InstIterator.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/Analysis/Dominators.h"
#include "llvm/Support/Debug.h"
#include "valueNumbering.h"

using namespace llvm;

//----------------------------------------------------------------------===
 //                              LCM Pass

//----------------------------------------------------------------------===

namespace {
  struct LCM : public FunctionPass {
    static char ID;
    LCM() : FunctionPass(ID) {}

    bool runOnFunction(Function &F);
    
    virtual void getAnalysisUsage(AnalysisUsage &AU) const {
      AU.addRequiredID(BreakCriticalEdgesID);
    }

    private:
    // add private members

  };

}  

char LCM::ID = 0;
static RegisterPass<LCM> X("lcm",
			    "Lazy Code Motion (CS526)",
			    false /* does not modify the CFG */,
			    false /* transformation, not just analysis */);


// Entry point for LCM function pass.
bool LCM::runOnFunction(Function &F) {

  RPO rpo(F);
  rpo.performVN();  // Value Numbering is implemented by this function

  
  
  // Example usage of getRepeatedValues()
  // .. 
  //std::vector<std::pair<uint32_t, uint32_t> > tmp = rpo.getRepeatedValues();
  //uint32_t bitVectorWidth = tmp.size();

  //for(std::vector<std::pair<uint32_t, uint32_t> >::iterator I = tmp.begin(), E = tmp.end(); I!=E; ++I)
  //   errs() << "VN-" << I->first << " Count-" << I->second << "\n";
  

  //for(inst_iterator I = inst_begin(F), E = inst_end(F); I!=E; ++I) 
     //errs() << *I << "\n";

  
    
  // TODO: change return value
  return true;

}
