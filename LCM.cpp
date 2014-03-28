//===--------------  LCM.cpp - Lazy Code Motion --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file was developed by the LLVM research group and is distributed under
// the University of Illinois Open Source License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//  
//      
//===----------------------------------------------------------------------===//

#define MYDEBUG 
#include "llvm/Transforms/Scalar.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/Pass.h"
#include "llvm/IR/Constants.h"
#include "llvm/Support/Debug.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallBitVector.h"
#include "llvm/IR/InstIterator.h"
#include "valueNumbering.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"

using namespace llvm;

namespace {
  
  struct dfva {
    SmallBitVector *In;  
    SmallBitVector *Out;  
    SmallBitVector *Gen;  
    SmallBitVector *Kill;  

    dfva() {}
  };
  
  struct LCM : public FunctionPass {
    static char ID;
    LCM() : FunctionPass(ID) {}

    bool runOnFunction(Function &F);
    DenseMap<BasicBlock*, dfva*> BBMap;
    uint32_t bitVectorWidth;
    
    virtual void getAnalysisUsage(AnalysisUsage &AU) const {
      AU.addRequired<DominatorTreeWrapperPass>();
      AU.setPreservesCFG();
      AU.addPreserved<DominatorTreeWrapperPass>();
      AU.addRequiredID(BreakCriticalEdgesID);
    }

    private:
      void performDFA(Function &) ;
      void initializeDFAFramework(Function &);
      void debugDFA();
      void dumpBBAttr(BasicBlock* BB, dfva* D, int = 0);
      void dumpSmallBitVector(SmallBitVector*);
      void performConstDFA(Function &);
      void performGlobalDFA(Function &);
      void calculateGen(BasicBlock*, SmallBitVector*);
      void calculateKill(BasicBlock*, SmallBitVector*);
      void TF(SmallBitVector*, SmallBitVector*, SmallBitVector*, SmallBitVector*);
  };

}  

char LCM::ID = 0;
static RegisterPass<LCM> X("lcm",
			    "Lazy Code Motion (CS526)",
			    false /* does not modify the CFG */,
			    false /* transformation, not just analysis */);

FunctionPass* doLCM() {return new LCM();}

/*******************************************************************
 * Function :   runOnFunction
 * Purpose  :   Entry point for LCM
********************************************************************/
bool LCM::runOnFunction(Function &F) 
{

  bool Changed = false;
  RPO rpo(F);
  rpo.performVN();  
  rpo.print();  

  std::vector<std::pair<uint32_t, uint32_t> > tmp = rpo.getRepeatedValues();
  bitVectorWidth = tmp.size();

  if(0 == bitVectorWidth) {
    dbgs() << "Nothing to do\n" ; 
    return Changed;
  }
  
  //for(std::vector<std::pair<uint32_t, uint32_t> >::iterator I = tmp.begin(), E = tmp.end(); I!=E; ++I)
  //   errs() << "VN-" << I->first << " Count-" << I->second << "\n";
  

  //for(inst_iterator I = inst_begin(F), E = inst_end(F); I!=E; ++I) 
     //errs() << *I << "\n";

  
    
  // TODO: change return value

  
  performDFA(F);

  return Changed;
}

/*******************************************************************
 * Function :   performDFA
 * Purpose  :   Entry point for DFA
********************************************************************/
void LCM::performDFA(Function &F) 
{
  initializeDFAFramework(F);
  
  performConstDFA(F);
  #ifdef MYDEBUG  
  debugDFA();
  #endif 
  performGlobalDFA(F);

}

/*******************************************************************
 * Function :   initializeDFAFramework
 * Purpose  :   Allocates the bitvectors 
********************************************************************/
void LCM::initializeDFAFramework(Function &F)
{
   ReversePostOrderTraversal<Function*> RPOT(&F);
   for (ReversePostOrderTraversal<Function*>::rpo_iterator I = RPOT.begin(),
      E = RPOT.end(); I != E; ++I) {
     BasicBlock* BB = *I;
     dfva *D    =   new dfva(); 
     D->In      =   new SmallBitVector(bitVectorWidth, false);    
     D->Out     =   new SmallBitVector(bitVectorWidth, false);    
     D->Gen     =   new SmallBitVector(bitVectorWidth, false);    
     D->Kill    =   new SmallBitVector(bitVectorWidth, false);    
     BBMap[BB] = D;
   }
}

/*******************************************************************
 * Function :   performConstDFA
 * Purpose  :   Calculate the Gen Kill for all the BBs
********************************************************************/
void LCM::performConstDFA(Function &F)
{
   ReversePostOrderTraversal<Function*> RPOT(&F);
   for (ReversePostOrderTraversal<Function*>::rpo_iterator I = RPOT.begin(),
      E = RPOT.end(); I != E; ++I) {
     BasicBlock* BB = *I;
     dfva *D  =   BBMap[BB];
     calculateGen(BB, D->Gen);
     calculateKill(BB, D->Kill);
   }
}

/*******************************************************************
 * Function :   calculateKill
 * Purpose  :   Calculate the Kill 
********************************************************************/
void LCM::calculateKill(BasicBlock* BB, SmallBitVector* BV)
{
  //TO DO
  for(unsigned VI=0; VI < BV->size(); VI++) {
    (*BV)[VI] = 1;
  }
}

/*******************************************************************
 * Function :   calculateGen
 * Purpose  :   Calculate the Antloc 
********************************************************************/
void LCM::calculateGen(BasicBlock* BB, SmallBitVector* BV)
{
  /*
  for (BasicBlock::iterator I = BB->begin(), E = BB->end(); I != E; ++I)
    Instruction* BBI = I;
    for (User::op_iterator OP = BBI->op_begin(), E = BBI->op_end(); OP != E; ++OP) {
      if(Instruction *Ins = dyn_cast<Instruction>(OP)) {

      }
    }
  }
  */
}

/*******************************************************************
 * Function :   performGlobalDFA
 * Purpose  :   Performing iterative DFA (backwards)
********************************************************************/
void LCM::performGlobalDFA(Function &F)
{
  bool change = false;
  int iterCount = 1;
  do {
    #ifdef MYDEBUG    
    dbgs() << "\t\t\tIteration " << iterCount << "\n";
    #endif
    iterCount++;
    change = false;
    for (po_iterator<BasicBlock *> I = po_begin(&F.getEntryBlock()),
                                 E = po_end(&F.getEntryBlock()); I != E; ++I) {
      BasicBlock* BB = *I;
      dfva* D = BBMap[BB];
      #ifdef MYDEBUG    
      dbgs() << "-------------------------------- \n";
      dbgs() << "Before : \n";
      dumpBBAttr(BB, D);
      #endif

      SmallBitVector* oldInBB   = D->In;
      SmallBitVector* oldOutBB  = D->Out;
      SmallBitVector* genBB     = D->Gen;
      SmallBitVector* killBB    = D->Kill;

      SmallBitVector* newOutBB = new SmallBitVector(bitVectorWidth, true);
      SmallBitVector* newInBB = new SmallBitVector(bitVectorWidth, false);
      
      // Calculate newOutBB as the meet of the In's of all the successors
      bool isExitBB = true;
      for (succ_iterator SI = succ_begin(BB), SE = succ_end(BB); SI != SE; SI++) {
        isExitBB = false;
        BasicBlock* SB  = *SI;
        SmallBitVector* inSB  =BBMap[SB]->In;
        (*newOutBB) &= (*inSB);
      }
      if(false == isExitBB) {
        delete oldOutBB;
        D->Out = newOutBB;
      }

      // Calculate the newInBB as the result of the transfer function
      TF(newInBB, genBB, D->Out, killBB);
      if((*newInBB) != (*oldInBB)) {
        change = true;
        delete oldInBB;
        D->In = newInBB;
      }
      #ifdef MYDEBUG
      dbgs() << "After : \n";
      dumpBBAttr(BB, D);
      #endif
    }
  } while(true == change);
}

/*******************************************************************
 * Function :   TF
 * Purpose  :   Transfer function
********************************************************************/
void LCM::TF(SmallBitVector* i, SmallBitVector*g, SmallBitVector*o, SmallBitVector*k) {
  (*i) = (*g) | ((*o) & ~(*k));
}

/*******************************************************************
 * Function :   dumpSmallBitVector
 * Purpose  :   
********************************************************************/
void LCM::dumpSmallBitVector(SmallBitVector* BV)
{
  dbgs() << "{";
  /*
  for (int VI = BV->find_first(); VI >= 0; VI = BV->find_next(VI)) {
    dbgs() << VI;
    if (BV->find_next(VI) >= 0)
      dbgs() << ' ';
  }
  */
  for(unsigned VI=0; VI < BV->size(); VI++) {
      dbgs() << (*BV)[VI];
      dbgs() << " ";
  }
  dbgs() << "}\n";

}

/*******************************************************************
 * Function :   debugDFA
 * Purpose  :   
********************************************************************/
void LCM::debugDFA()
{
  for (DenseMap<BasicBlock*, dfva*> ::iterator
      I = BBMap.begin(), E = BBMap.end(); I != E; ++I) {
    BasicBlock* BB  = I->first;    
    dfva* D         = I->second;    
    dumpBBAttr(BB, D, 1);
  }
}

/*******************************************************************
 * Function :   dumpBBAttr
 * Purpose  :   Dumps the attributes of a BB
********************************************************************/
void LCM::dumpBBAttr(BasicBlock* BB, dfva* D, int verbose )
{
    dbgs() << "===============\n";
    dbgs() << "Basic Block :\n";
    dbgs() << "===============\n";
    BB->printAsOperand(dbgs(),false);
    if(verbose) {
      dbgs() << *BB;
    }
    dbgs() << "\n";
    dbgs() << "IN: ";   dumpSmallBitVector(D->In);
    dbgs() << "OUT: ";   dumpSmallBitVector(D->Out);
    dbgs() << "GEN: ";   dumpSmallBitVector(D->Gen);
    dbgs() << "KILL: ";   dumpSmallBitVector(D->Kill);
    dbgs() << "\n";
}
