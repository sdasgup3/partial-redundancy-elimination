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
  
  // enum for the bitVectors associated with each basic block
  enum bitVectors {
   ANTLOC = 0,
   TRANSP,
   ANTIN, 
   ANTOUT,
   EARLIN,
   EARLOUT,
   DELAYIN,
   DELAYOUT,
   LATESTIN,
   ISOLIN,
   ISOLOUT,
   OPT,
   REDN,
   TOTALBITVECTORS // = 13
  };
  
  struct LCM : public FunctionPass {
    
    typedef std::vector<SmallBitVector*> dfva;
    
    static char ID;
    LCM() : FunctionPass(ID) {}
    bool runOnFunction(Function &F);
    virtual void getAnalysisUsage(AnalysisUsage &AU) const {
      //AU.addRequired<DominatorTreeWrapperPass>();
      AU.setPreservesCFG();
      //AU.addPreserved<DominatorTreeWrapperPass>();
      AU.addRequiredID(BreakCriticalEdgesID);
    }
    void releaseMemory();

    private:
      Function* Func;
      RPO* rpo;

      // Maps each Basic Block to a vector of SmallBitVectors, each of which
      // represents a property as defined in bitVectors enum 
      DenseMap<BasicBlock*, dfva*> BBMap;
    
      // Each bitVector is only as wide as the number of values which occur 
      // more than once in the program
      uint32_t bitVectorWidth;

      // functions 
      void performDFA() ;
      void initializeDFAFramework();
      void debugDFA();
      void dumpBBAttr(BasicBlock* BB, dfva* D, int = 0);
      void dumpSmallBitVector(SmallBitVector*);
      void performConstDFA();
      void performGlobalDFA();
      
      void callFramework(uint32_t out, uint32_t in, std::vector<uint32_t> alpha, std::vector<uint32_t> beta, std::vector<uint32_t> gamma, bool meetOp, bool init, bool direction);
      void calculateLatestIn();
      void calculateOptAndRedn();
      SmallBitVector getSBVForExpression(std::vector<uint32_t> input, BasicBlock* BB);
      SmallBitVector getSBVForElement(uint32_t num, BasicBlock* BB);
      void printFlowEquations();

      SmallBitVector calculateAntloc(BasicBlock*);
      SmallBitVector calculateTrans(BasicBlock*);
  };
}  

char LCM::ID = 0;
static RegisterPass<LCM> X("lcm",
			    "Lazy Code Motion (CS526)",
			    false /* does not modify the CFG */,
			    false /* transformation, not just analysis */);

/*******************************************************************
 * Function :   runOnFunction
 * Purpose  :   Entry point for LCM
********************************************************************/
bool LCM::runOnFunction(Function &F) 
{
  Func = &F;
  bool Changed = false;
  rpo = new RPO(F);
  rpo->performVN();  

  rpo->print();  
  
  bitVectorWidth = rpo->getRepeatedValues().size();

  if(0 == bitVectorWidth) {
    dbgs() << "Nothing to do\n" ; 
    return Changed;
  }
  
  performDFA();
  releaseMemory();

  // TODO: change return value
  return Changed;
}

/*******************************************************************
 * Function :   performDFA
 * Purpose  :   Entry point for DFA
********************************************************************/
void LCM::performDFA() {

  initializeDFAFramework();
  
  // This function is responsible for calculating the local sets for each basic
  // block, namely the transp and antloc bitvectors
  performConstDFA();
  
  #ifdef MYDEBUG  
  //debugDFA();
  #endif 
  
  // This function calls the various data flow equations of LCM
  performGlobalDFA();

  printFlowEquations();
}

/*******************************************************************
 * Function :   initializeDFAFramework
 * Purpose  :   Allocates the bitvectors 
********************************************************************/
void LCM::initializeDFAFramework() {

  for (Function::iterator BB = Func->begin(), E = Func->end(); BB != E; ++BB) {

    dfva* dfvaInstance = new dfva();
    for(uint32_t i = 0; i < TOTALBITVECTORS; i++) 
      dfvaInstance->push_back(new SmallBitVector(bitVectorWidth, false));
    
    BBMap[BB] = dfvaInstance;
  }
}

/*******************************************************************
 * Function :   performConstDFA
 * Purpose  :   Calculate the Gen Kill for all the BBs
********************************************************************/
void LCM::performConstDFA()
{

  SmallBitVector random(bitVectorWidth, false);

  /*
  for(unsigned VI=0; VI < bitVectorWidth; VI++) 
    if(VI%2)    
      random[VI] = true;
  */


  dfva* dfvaInstance;
  //uint32_t k=0;
  //rpo->print();
  for (Function::iterator BB = Func->begin(), E = Func->end(); BB != E; ++BB) {
    dfvaInstance = BBMap[BB];

    *((*dfvaInstance)[ANTLOC]) = calculateAntloc(BB);
    *((*dfvaInstance)[TRANSP]) = calculateTrans(BB);
    /*
    if((++k)%3)
      *((*dfvaInstance)[ANTLOC]) = random;

    if(k%2)
      *((*dfvaInstance)[TRANSP]) = random;
    */
  }
}

/*******************************************************************
 * Function :   calculateTrans
 * Purpose  :   Calculate the Trans 
********************************************************************/
SmallBitVector LCM::calculateTrans(BasicBlock* BB)
{
  dbgs() << "Finding Trans BB\n";
  BB->printAsOperand(dbgs(),false);

  SmallBitVector returnValue(bitVectorWidth, true);

  for (BasicBlock::iterator I = BB->begin(), E = BB->end(); I != E; ++I) {
    Instruction* BBI = I;
    uint32_t  VI  = rpo->getBitVectorPosition(BBI);  
    if(VI >= bitVectorWidth) 
      continue;
    dbgs() << "\tInstruction: " << *BBI << " Value " << VI  << " \n";
    Value* V = rpo->getLeader(BBI);
    Instruction* LI =  cast<Instruction>(V);
    if(NULL == LI) {
      continue;
    }
    dbgs() << "\tLeader Instruction: " << *LI << " \n";
    bool isAnyOpDef = false;
    for (User::op_iterator OP = LI->op_begin(), E = LI->op_end(); OP != E; ++OP) {
      if(Instruction *Ins = dyn_cast<Instruction>(OP)) {
        isAnyOpDef = true;
        if(BB != Ins->getParent()) {
          dbgs() << " Transparent \n";
          returnValue[VI] = 0;
        }
      }
    }
    if(false == isAnyOpDef) {
      dbgs() << " Transparent \n";
      returnValue[VI] = 0;
    }
  }
  return returnValue;
}

/*******************************************************************
 * Function :   calculateAntloc
 * Purpose  :   Calculate the Antloc 
********************************************************************/
SmallBitVector LCM::calculateAntloc(BasicBlock* BB)
{
  dbgs() << "Finding Antloc BB\n";
  BB->printAsOperand(dbgs(),false);

  SmallBitVector returnValue(bitVectorWidth, false);

  for (BasicBlock::iterator I = BB->begin(), E = BB->end(); I != E; ++I) {
    Instruction* BBI = I;
    uint32_t  VI  = rpo->getBitVectorPosition(BBI);  
    if(VI >= bitVectorWidth) 
      continue;
    dbgs() << "\tInstruction: " << *BBI << " Value " << VI << "Size" << bitVectorWidth<< " \n";
    bool isAnyOpDef = false;
    for (User::op_iterator OP = BBI->op_begin(), E = BBI->op_end(); OP != E; ++OP) {
      if(Instruction *Ins = dyn_cast<Instruction>(OP)) {
        isAnyOpDef = true;
        dbgs() << *Ins << "\n";
        if(BB != Ins->getParent()) {
          dbgs() << " Antloc \n";
          returnValue[VI] = 1;  
        }
      }
    }
    if(false == isAnyOpDef) {
      dbgs() << " Antloc \n";
      returnValue[VI] = 1;  
    }
  }
  return returnValue;
}

// See desciption of getSBVForExpression function
SmallBitVector LCM::getSBVForElement(uint32_t num, BasicBlock* BB) {
    
  dfva* dfvaInstance = BBMap[BB];
  uint32_t pos;
  bool compliment = false;

  if(num < TOTALBITVECTORS)
    pos = num;
  else {
    pos = num - TOTALBITVECTORS;
    compliment = true;
  }

  SmallBitVector returnValue = *(*dfvaInstance)[pos];
  if(compliment)
     return ~returnValue;

  return returnValue;
}

// For a basic block, this function takes in an expression (in the form of
// std::vector) and returns the SmallBitVector computed for that expression.
// The vector values are inserted based on the enum definition 
//
//    e.g. ~ANTLOC U ~TRANSP .The corresponding input vector would be :
//     13 - for ~ANTLOC (0 for ANTLOC, plus 13 since it is compliment)
//     0 - for union operation
//     14 - for ~ TRANSP (1 for TRANSP, plus 13 since it is compliment)
//
// Note that 13 is used in example above since TOTALBITVECTORS enum = 13
// The individual values of an expression (~ANTLOC and ~TRANSP) are computed by
// the function getSBVForElement()
SmallBitVector LCM::getSBVForExpression(std::vector<uint32_t> input, BasicBlock* BB) {
    
  SmallBitVector returnValue(bitVectorWidth, false);
  bool first = true;

  for(uint32_t I = 0; I < input.size(); ++I) {
    if(first) {
      first = false;
      returnValue = getSBVForElement(input[I], BB);
    }  

    // this position in the vector is for an union/intersection operator
    // 1 = intersection, 0 = union
    if(I%2 != 0) {
      if(input[I])    
        returnValue &= getSBVForElement(input[++I], BB);
      else
        returnValue |= getSBVForElement(input[++I], BB);
    }
  }

  return returnValue;
}

// This is the generalized data-flow equation solving framework. This works for
// all the equations involved in Lazy Code Motion. Inputs are the following :
// out, in : enum values for the SmallBitVector associated with the data flow
// property being computed at In[B] and Out[B]
// alpha, beta, gamma : std::vectors which describe the expressions that form
// the GEN and KILL of the data flow equations
// meetOp : 1=Intersection, 0=Union
// init : denotes value for start/exit node as the case may be
// direction : 1=fwd. data flow, 0=backward data flow
//
//  e.g for DELAY
//  in : DELAYIN
//  out : DELAYOUT
//  alpha : empty vector (equivalent SmallBitVector would be allFalse)
//  beta : 13+0 (equivalent SmallBitVector would represent ~ANTLOC)
//  gamma : ANTIN, 1, EARLIN (equivalent SmallBitVector would represent ANTIN <intersection> EARLIN)
//  meetOp : 1 (intersection)
//  init : 0 
//  direction : 1 (fwd.)
//
void LCM::callFramework(uint32_t out, uint32_t in, std::vector<uint32_t> alpha, std::vector<uint32_t> beta, std::vector<uint32_t> gamma, bool meetOp, bool init, bool direction) {

  bool Changed = false;
  SmallBitVector allTrue(bitVectorWidth, true);
  SmallBitVector allFalse(bitVectorWidth, false);
                                       
  // direction == 1 : Forward Data Flow
  if(direction) {

    do {
      Changed = false;
      ReversePostOrderTraversal<Function*> RPOT(Func);
      
      for (ReversePostOrderTraversal<Function*>::rpo_iterator I = RPOT.begin(), E = RPOT.end(); I != E; ++I) {
        BasicBlock* BB = *I;
        dfva* dfvaInstance = BBMap[BB];
        SmallBitVector* outVector = (*dfvaInstance)[out];
        SmallBitVector* inVector = (*dfvaInstance)[in];

        // bitVector for in[B] of start node
        SmallBitVector& initVector = (init ? allTrue : allFalse);

        // this vector would be initialized accordingly later by the 
        // first predecessor while taking a meet over predecessors
        SmallBitVector meetOverPreds(bitVectorWidth, false);

        // go over predecessors and take a meet
        bool first = true;
        for(pred_iterator PI = pred_begin(BB), PE = pred_end(BB); PI!=PE; ++PI) {
          
          dfva* predDfvaInstance = BBMap[*PI];
          
          if(first) {
           first = false;
           meetOverPreds = *(*predDfvaInstance)[out];
          }
          else {
            if(meetOp)
              meetOverPreds &= *(*predDfvaInstance)[out];
            else
              meetOverPreds |= *(*predDfvaInstance)[out];
          }
        }
   
        // no predecessor, this is the start block s.
        if(first)
          meetOverPreds = initVector;

        // 1st data flow eq. 'In' as a function of 'Out'
        *(inVector) = getSBVForExpression(gamma, BB) | meetOverPreds;   
        
        SmallBitVector oldOutVector = *(outVector);
        // 2nd data flow eq. 'Out' as a function of 'In'
        *(outVector) = getSBVForExpression(alpha, BB) | (*(inVector) & getSBVForExpression(beta, BB));  
        SmallBitVector newOutVector = *(outVector);
        if(oldOutVector != newOutVector)
          Changed = true;
      }
    } while(Changed);
  
  }

  // Backward data-flow analysis
  else {
    
    do {
      Changed = false;
      
      for (po_iterator<BasicBlock *> I = po_begin(&Func->getEntryBlock()), E = po_end(&Func->getEntryBlock()); I != E; ++I) {
        BasicBlock* BB = *I;
        dfva* dfvaInstance = BBMap[BB];
        SmallBitVector* outVector = (*dfvaInstance)[out];
        SmallBitVector* inVector = (*dfvaInstance)[in];

        // bitVector for out[B] of exit node
        SmallBitVector& initVector = (init ? allTrue : allFalse);

        // this vector would be initialized accordingly later by the 
        // first successor while taking a meet over successors
        SmallBitVector meetOverSucc(bitVectorWidth, false);

        // go over successors and take a meet
        bool first = true;
        for (succ_iterator SI = succ_begin(BB), SE = succ_end(BB); SI != SE; SI++) {
          
          dfva* succDfvaInstance = BBMap[*SI];
          
          if(first) {
           first = false;
           meetOverSucc = *(*succDfvaInstance)[in];
          }
          else {
            if(meetOp)
              meetOverSucc &= *(*succDfvaInstance)[in];
            else
              meetOverSucc |= *(*succDfvaInstance)[in];
          }
        }
   
        // no successor, this is the exit block e.
        if(first)
          meetOverSucc = initVector;

        // 1st data flow eq. 'Out' as a function of 'In'
        *(outVector) = meetOverSucc; 

        SmallBitVector oldInVector = *(inVector);
        // 2nd data flow eq. 'In' as a function of 'Out'
        *(inVector) = getSBVForExpression(alpha, BB) | (*(outVector) & getSBVForExpression(beta, BB));  
        SmallBitVector newInVector = *(inVector);
        if(oldInVector != newInVector)
          Changed = true;
      }
    } while(Changed);
  
  }
}

// This function calculates the OPT and REDN SmallBitVectors for each Basic Block
void LCM::calculateOptAndRedn() {
  
  dfva* dfvaInstance;
  for (Function::iterator BB = Func->begin(), E = Func->end(); BB != E; ++BB) {
    dfvaInstance = BBMap[BB];

    // eq. for OPT
    *(*dfvaInstance)[OPT] = *(*dfvaInstance)[LATESTIN] & ~(*(*dfvaInstance)[ISOLOUT]);

    // eq. for REDN
    *(*dfvaInstance)[REDN] = *(*dfvaInstance)[ANTLOC] & ~(*(*dfvaInstance)[LATESTIN] & *(*dfvaInstance)[ISOLOUT]);
  }
}

// This function calculates the LATESTIN SmallBitVector for each Basic Block
void LCM::calculateLatestIn() {

  dfva* dfvaInstance;
  for (Function::iterator BB = Func->begin(), E = Func->end(); BB != E; ++BB) {
    dfvaInstance = BBMap[BB];
    
    bool first = true;
    SmallBitVector meetOverSucc(bitVectorWidth, false);  

    // go over successors and take a meet
    for (succ_iterator SI = succ_begin(BB), SE = succ_end(BB); SI != SE; SI++) {
          
      dfva* succDfvaInstance = BBMap[*SI];
          
      if(first) {
        first = false;
        meetOverSucc = *(*succDfvaInstance)[DELAYIN];
      }
      else 
        meetOverSucc &= *(*succDfvaInstance)[DELAYIN];
    }

    // eq. for LATESTIN
    *(*dfvaInstance)[LATESTIN] = *(*dfvaInstance)[DELAYIN] & (*(*dfvaInstance)[ANTLOC] | ~meetOverSucc);
  }
}


// This function calls the data-flow framework with differnt 
// initializations for different properties
void LCM::performGlobalDFA() {
  
  std::vector<uint32_t> alpha, beta, gamma;

  // -- ANTICIPATABLE
  alpha.push_back(ANTLOC);
  beta.push_back(TRANSP);
  callFramework(ANTOUT, ANTIN, alpha, beta, gamma, 1, 0, 0);
  alpha.clear();
  beta.clear();

  // -- EARLIEST
  alpha.push_back(TOTALBITVECTORS + TRANSP);
  beta.push_back(TOTALBITVECTORS + ANTIN);
  callFramework(EARLOUT, EARLIN, alpha, beta, gamma, 0, 1, 1);
  alpha.clear();
  beta.clear();

  // -- DELAY
  beta.push_back(TOTALBITVECTORS + ANTLOC);
  gamma.push_back(ANTIN);
  gamma.push_back(1);
  gamma.push_back(EARLIN);
  callFramework(DELAYOUT, DELAYIN, alpha, beta, gamma, 1, 0, 1);
  beta.clear();
  gamma.clear();

  // LATESTIN does not fit into the data-flow framework. It is computed in a
  // separate function
  calculateLatestIn();

  // -- ISOLATEDNESS
  alpha.push_back(LATESTIN);
  beta.push_back(TOTALBITVECTORS + ANTLOC);
  callFramework(ISOLOUT, ISOLIN, alpha, beta, gamma, 1, 0, 0);
  alpha.clear();
  beta.clear();

  // calculating optimal and redundant points
  calculateOptAndRedn();
}

void LCM::dumpSmallBitVector(SmallBitVector* BV) {

  errs() << "{";
  for(unsigned VI=0; VI < BV->size(); VI++) {
      errs() << (*BV)[VI];
      errs() << " ";
  }
  errs() << "}\n";
}

/*******************************************************************
 * Function :   debugDFA
 * Purpose  :   
********************************************************************/
void LCM::debugDFA() {
/*
  for (DenseMap<BasicBlock*, dfva*> ::iterator
      I = BBMap.begin(), E = BBMap.end(); I != E; ++I) {
    BasicBlock* BB  = I->first;    
    dfva* D         = I->second;    
    dumpBBAttr(BB, D, 1);
  }*/
}

/*******************************************************************
 * Function :   dumpBBAttr
 * Purpose  :   Dumps the attributes of a BB
********************************************************************/
void LCM::dumpBBAttr(BasicBlock* BB, dfva* D, int verbose ) {
/*    
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
  dbgs() << "\n";*/
}

void LCM::releaseMemory() {

  // TODO
  // destroy the DFA framework and free all allocated heap memory
}

void LCM::printFlowEquations() {
  
  dfva* dfvaInstance;
  for (Function::iterator BB = Func->begin(), E = Func->end(); BB != E; ++BB) {
    dfvaInstance = BBMap[BB];
    errs() << *BB << "\n";
    errs() << "-----\n";
    for(uint32_t i = 0; i < TOTALBITVECTORS; i++) 
      dumpSmallBitVector((*dfvaInstance)[i]);
    errs() << "-----\n"; 
  }
}
