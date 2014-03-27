#include "llvm/Transforms/Scalar.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/Pass.h"
#include "llvm/IR/Constants.h"
#include "llvm/Support/Debug.h"
#include "llvm/IR/InstIterator.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/IR/Dominators.h"
#include "llvm/Support/Debug.h"
#include "valueNumbering.h"

//----------------------------------------------------------------------===
 //                     ValueTable Internal Functions

//----------------------------------------------------------------------===
 Expression::ExpressionOpcode ValueTable::getOpcode(BinaryOperator* BO) {
   switch(BO->getOpcode()) {
   default: // THIS SHOULD NEVER HAPPEN
     llvm_unreachable("Binary operator with unknown opcode?");
   case Instruction::Add:  return Expression::ADD;
   case Instruction::FAdd: return Expression::FADD;
   case Instruction::Sub:  return Expression::SUB;
   case Instruction::FSub: return Expression::FSUB;
   case Instruction::Mul:  return Expression::MUL;
   case Instruction::FMul: return Expression::FMUL;
   case Instruction::UDiv: return Expression::UDIV;
   case Instruction::SDiv: return Expression::SDIV;
   case Instruction::FDiv: return Expression::FDIV;
   case Instruction::URem: return Expression::UREM;
   case Instruction::SRem: return Expression::SREM;
   case Instruction::FRem: return Expression::FREM;
   case Instruction::Shl:  return Expression::SHL;
   case Instruction::LShr: return Expression::LSHR;
   case Instruction::AShr: return Expression::ASHR;
   case Instruction::And:  return Expression::AND;
   case Instruction::Or:   return Expression::OR;
   case Instruction::Xor:  return Expression::XOR;
   }
 }
 
 Expression::ExpressionOpcode ValueTable::getOpcode(CmpInst* C) {
   if (isa<ICmpInst>(C)) {
     switch (C->getPredicate()) {
     default:  // THIS SHOULD NEVER HAPPEN
       llvm_unreachable("Comparison with unknown predicate?");
     case ICmpInst::ICMP_EQ:  return Expression::ICMPEQ;
     case ICmpInst::ICMP_NE:  return Expression::ICMPNE;
     case ICmpInst::ICMP_UGT: return Expression::ICMPUGT;
     case ICmpInst::ICMP_UGE: return Expression::ICMPUGE;
     case ICmpInst::ICMP_ULT: return Expression::ICMPULT;
     case ICmpInst::ICMP_ULE: return Expression::ICMPULE;
     case ICmpInst::ICMP_SGT: return Expression::ICMPSGT;
     case ICmpInst::ICMP_SGE: return Expression::ICMPSGE;
     case ICmpInst::ICMP_SLT: return Expression::ICMPSLT;
     case ICmpInst::ICMP_SLE: return Expression::ICMPSLE;
     }
   } else {
     switch (C->getPredicate()) {
     default: // THIS SHOULD NEVER HAPPEN
       llvm_unreachable("Comparison with unknown predicate?");
     case FCmpInst::FCMP_OEQ: return Expression::FCMPOEQ;
     case FCmpInst::FCMP_OGT: return Expression::FCMPOGT;
     case FCmpInst::FCMP_OGE: return Expression::FCMPOGE;
     case FCmpInst::FCMP_OLT: return Expression::FCMPOLT;
     case FCmpInst::FCMP_OLE: return Expression::FCMPOLE;
     case FCmpInst::FCMP_ONE: return Expression::FCMPONE;
     case FCmpInst::FCMP_ORD: return Expression::FCMPORD;
     case FCmpInst::FCMP_UNO: return Expression::FCMPUNO;
     case FCmpInst::FCMP_UEQ: return Expression::FCMPUEQ;
     case FCmpInst::FCMP_UGT: return Expression::FCMPUGT;
     case FCmpInst::FCMP_UGE: return Expression::FCMPUGE;
     case FCmpInst::FCMP_ULT: return Expression::FCMPULT;
     case FCmpInst::FCMP_ULE: return Expression::FCMPULE;
     case FCmpInst::FCMP_UNE: return Expression::FCMPUNE;
     }
   }
 }
 
 Expression::ExpressionOpcode ValueTable::getOpcode(CastInst* C) {
   switch(C->getOpcode()) {
   default: // THIS SHOULD NEVER HAPPEN
     llvm_unreachable("Cast operator with unknown opcode?");
   case Instruction::Trunc:    return Expression::TRUNC;
   case Instruction::ZExt:     return Expression::ZEXT;
   case Instruction::SExt:     return Expression::SEXT;
   case Instruction::FPToUI:   return Expression::FPTOUI;
   case Instruction::FPToSI:   return Expression::FPTOSI;
   case Instruction::UIToFP:   return Expression::UITOFP;
   case Instruction::SIToFP:   return Expression::SITOFP;
   case Instruction::FPTrunc:  return Expression::FPTRUNC;
   case Instruction::FPExt:    return Expression::FPEXT;
   case Instruction::PtrToInt: return Expression::PTRTOINT;
   case Instruction::IntToPtr: return Expression::INTTOPTR;
   case Instruction::BitCast:  return Expression::BITCAST;
   }
 }
 
 Expression ValueTable::create_expression(CallInst* C) {
   Expression e;
 
   e.type = C->getType();
   e.opcode = Expression::CALL;
 
   e.varargs.push_back(lookup(C->getCalledFunction()));
   for (CallInst::op_iterator I = C->op_begin()+1, E = C->op_end();
        I != E; ++I)
     e.varargs.push_back(lookup(*I));
 
   return e;
 }
 
 Expression ValueTable::create_expression(BinaryOperator* BO) {
   Expression e;

   e.varargs.push_back(lookup(BO->getOperand(0)));
   e.varargs.push_back(lookup(BO->getOperand(1)));
   e.type = BO->getType();
   e.opcode = getOpcode(BO);
 
   return e;
 }
 
 Expression ValueTable::create_expression(CmpInst* C) {
   Expression e;
 
   e.varargs.push_back(lookup(C->getOperand(0)));
   e.varargs.push_back(lookup(C->getOperand(1)));
   e.type = C->getType();
   e.opcode = getOpcode(C);
 
   return e;
 }
 
 Expression ValueTable::create_expression(CastInst* C) {
   Expression e;
 
   e.varargs.push_back(lookup(C->getOperand(0)));
   e.type = C->getType();
   e.opcode = getOpcode(C);
 
   return e;
 }
 
 Expression ValueTable::create_expression(ShuffleVectorInst* S) {
   Expression e;
 
   e.varargs.push_back(lookup(S->getOperand(0)));
   e.varargs.push_back(lookup(S->getOperand(1)));
   e.varargs.push_back(lookup(S->getOperand(2)));
   e.type = S->getType();
   e.opcode = Expression::SHUFFLE;
 
   return e;
 }
 
 Expression ValueTable::create_expression(ExtractElementInst* E) {
   Expression e;
 
   e.varargs.push_back(lookup(E->getOperand(0)));
   e.varargs.push_back(lookup(E->getOperand(1)));
   e.type = E->getType();
   e.opcode = Expression::EXTRACT;
 
   return e;
 }
 
 Expression ValueTable::create_expression(InsertElementInst* I) {
   Expression e;
 
   e.varargs.push_back(lookup(I->getOperand(0)));
   e.varargs.push_back(lookup(I->getOperand(1)));
   e.varargs.push_back(lookup(I->getOperand(2)));
   e.type = I->getType();
   e.opcode = Expression::INSERT;
 
   return e;
 }
 
 Expression ValueTable::create_expression(SelectInst* I) {
   Expression e;
 
   e.varargs.push_back(lookup(I->getCondition()));
   e.varargs.push_back(lookup(I->getTrueValue()));
   e.varargs.push_back(lookup(I->getFalseValue()));
   e.type = I->getType();
   e.opcode = Expression::SELECT;
 
   return e;
 }
 
 Expression ValueTable::create_expression(GetElementPtrInst* G) {
   Expression e;
 
   e.varargs.push_back(lookup(G->getPointerOperand()));
   e.type = G->getType();
   e.opcode = Expression::GEP;
 
   for (GetElementPtrInst::op_iterator I = G->idx_begin(), E = G->idx_end(); I != E; ++I)
     e.varargs.push_back(lookup(*I));
 
   return e;
 }
 
 Expression ValueTable::create_expression(ExtractValueInst* E) {
   Expression e;
 
   e.varargs.push_back(lookup(E->getAggregateOperand()));
   for (ExtractValueInst::idx_iterator II = E->idx_begin(), IE = E->idx_end(); II != IE; ++II)
     e.varargs.push_back(*II);

   e.type = E->getType();
   e.opcode = Expression::EXTRACTVALUE;
 
   return e;
 }
 
 Expression ValueTable::create_expression(InsertValueInst* E) {
   Expression e;
 
   e.varargs.push_back(lookup(E->getAggregateOperand()));
   e.varargs.push_back(lookup(E->getInsertedValueOperand()));
   for (InsertValueInst::idx_iterator II = E->idx_begin(), IE = E->idx_end(); II != IE; ++II)
     e.varargs.push_back(*II);
   e.type = E->getType();
   e.opcode = Expression::INSERTVALUE;
 
   return e;
 }

//----------------------------------------------------------------------===
 //                     ValueTable External Functions

//----------------------------------------------------------------------===
 
 /// add - Insert a value into the table with a specified value number.
 void ValueTable::add(Value *V, uint32_t num) {
   valueNumbering[V] = num;
 }
 
 /// computeNumber - Returns the value number for the specified value, assigning
 /// it a new number if it did not have one before.
 uint32_t ValueTable::computeNumber(Value *V) {
   if (uint32_t v = valueNumbering[V])
     return v;
   else if (uint32_t v= constantsNumbering[V])
     return v;
 
   if (!isa<Instruction>(V)) {
     constantsNumbering[V] = nextValueNumber;
     return nextValueNumber++;
   }
   
   Instruction* I = cast<Instruction>(V);
   Expression exp;
   switch (I->getOpcode()) {
     case Instruction::Add:
     case Instruction::FAdd:
     case Instruction::Sub:
     case Instruction::FSub:
     case Instruction::Mul:
     case Instruction::FMul:
     case Instruction::UDiv:
     case Instruction::SDiv:
     case Instruction::FDiv:
     case Instruction::URem:
     case Instruction::SRem:
     case Instruction::FRem:
     case Instruction::Shl:
     case Instruction::LShr:
     case Instruction::AShr:
     case Instruction::And:
     case Instruction::Or :
     case Instruction::Xor:
       exp = create_expression(cast<BinaryOperator>(I));
       break;
     case Instruction::ICmp:
     case Instruction::FCmp:
       exp = create_expression(cast<CmpInst>(I));
       break;
     case Instruction::Trunc:
     case Instruction::ZExt:
     case Instruction::SExt:
     case Instruction::FPToUI:
     case Instruction::FPToSI:
     case Instruction::UIToFP:
     case Instruction::SIToFP:
     case Instruction::FPTrunc:
     case Instruction::FPExt:
     case Instruction::PtrToInt:
     case Instruction::IntToPtr:
     case Instruction::BitCast:
       exp = create_expression(cast<CastInst>(I));
       break;
     case Instruction::Select:
       exp = create_expression(cast<SelectInst>(I));
       break;
     case Instruction::ExtractElement:
       exp = create_expression(cast<ExtractElementInst>(I));
       break;
     case Instruction::InsertElement:
       exp = create_expression(cast<InsertElementInst>(I));
       break;
     case Instruction::ShuffleVector:
       exp = create_expression(cast<ShuffleVectorInst>(I));
       break;
     case Instruction::ExtractValue:
       exp = create_expression(cast<ExtractValueInst>(I));
       break;
     case Instruction::InsertValue:
       exp = create_expression(cast<InsertValueInst>(I));
       break;      
     case Instruction::GetElementPtr:
       exp = create_expression(cast<GetElementPtrInst>(I));
       break;
     default:
       valueNumbering[V] = nextValueNumber;
       return nextValueNumber++;
   }
 
   uint32_t& e = expressionNumbering[exp];
   if (!e) {
     e = nextValueNumber++;
     leaderBoard[e] = V;
   }
   
   valueNumbering[V] = e;
   
   return e;
 }
 
 /// lookup - Returns the value number of the specified value. Returns 0 if
 /// the value has not yet been numbered.
 uint32_t ValueTable::lookup(Value *V) {
   if (!isa<Instruction>(V)) {
     if (!constantsNumbering.count(V))
       constantsNumbering[V] = nextValueNumber++;
     return constantsNumbering[V];
   }
   
   return valueNumbering[V];
 }
 
 /// clear - Remove all entries from the ValueTable
 void ValueTable::clear() {
   valueNumbering.clear();
   leaderBoard.clear();
   expressionNumbering.clear();
   constantsNumbering.clear();
   nextValueNumber = 1;
 }
 
 void ValueTable::clearExpressions() {
   expressionNumbering.clear();
   constantsNumbering.clear();
   nextValueNumber = 1;
 }
 
 /// erase - Remove a value from the value numbering
 void ValueTable::erase(Value *V) {
   valueNumbering.erase(V);
 }
 
 /// verifyRemoved - Verify that the value is removed from all internal
 /// data structures.
 void ValueTable::verifyRemoved(const Value *V) const {
   for (DenseMap<Value*, uint32_t>::const_iterator I = valueNumbering.begin(), E = valueNumbering.end(); I != E; ++I)
     assert(I->first != V && "Inst still occurs in value numbering map!");
 }

//----------------------------------------------------------------------===
//              RPO external functions    

//----------------------------------------------------------------------===

void RPO::performVN() {

  ReversePostOrderTraversal<Function*> RPOT(&F);
  bool done = false;

  // the RPO algorithm from SCC paper
  while (!done) {
    done = true;
    VT.clearExpressions();
    for (ReversePostOrderTraversal<Function*>::rpo_iterator I = RPOT.begin(), E = RPOT.end(); I != E; ++I) {
      BasicBlock* BB = *I;
      for (BasicBlock::iterator BI = BB->begin(), BE = BB->end(); BI != BE; ++BI) {
        uint32_t origVN = VT.lookup(BI);
        uint32_t newVN = VT.computeNumber(BI);
        if (origVN != newVN)
          done = false;
      }
    }
  }

  handleSpecialCases();
}

void RPO::handleSpecialCases() {

  for(inst_iterator I = inst_begin(F), E = inst_end(F); I!=E;) {

   if(I->getOpcode() == Instruction::Or || I->getOpcode() == Instruction::And) {
    uint32_t vn1 = VT.lookup(I->getOperand(0));
    uint32_t vn2 = VT.lookup(I->getOperand(1));

    if(vn1 == vn2) {
      I->replaceAllUsesWith(I->getOperand(0));
      Instruction* OldInst = &*I;
      ++I;
      OldInst->eraseFromParent();
      continue;
    }
   }

   if(ICmpInst *Cmp = dyn_cast<ICmpInst>(&*I)) {
    uint32_t vn1 = VT.lookup(I->getOperand(0));
    uint32_t vn2 = VT.lookup(I->getOperand(1));

    if(vn1 == vn2) {
      if(Cmp->getPredicate() == CmpInst::ICMP_EQ || Cmp->getPredicate() == CmpInst::ICMP_NE) {
        Cmp->replaceAllUsesWith(ConstantInt::get(Cmp->getType(), Cmp->getPredicate() == CmpInst::ICMP_EQ));
        Instruction* OldInst = &*I;
        ++I;
        OldInst->eraseFromParent();
        continue;
      }
    }
   }

   ++I;
  }
}

uint32_t RPO::getNumberForValue(Value *V) {
  return VT.lookup(V);
}

void RPO::getEqualValues(Value* V, SmallVectorImpl<Value*> &equalValues) {
    
  uint32_t vn = VT.lookup(V);
  getEqualValues(vn, equalValues);
}

void RPO::getEqualValues(uint32_t VN, SmallVectorImpl<Value*> &equalValues) {
  
  for (DenseMap<Value*, uint32_t>::const_iterator I = VT.valueNumbering.begin(), E = VT.valueNumbering.end(); I != E; ++I) {
    if(I->second == VN)
      equalValues.push_back(I->first);
  }
}

Value* RPO::getLeader(Value* V) {

  SmallVector<Value*, 8> equalValues;
  getEqualValues(V, equalValues);
  assert(equalValues.size() != 1 && "Request for leader of Value which only has a single occurence in the function");
  
  uint32_t vn = VT.lookup(V);
  return VT.leaderBoard[vn];
}

std::vector<std::pair<uint32_t, uint32_t> > RPO::getRepeatedValues() {
  
  std::vector<std::pair<uint32_t, uint32_t> > valueCountVector;
  SmallVector<Value*, 8> equalValues;

  for (DenseMap<uint32_t, Value*>::const_iterator I = VT.leaderBoard.begin(), E = VT.leaderBoard.end(); I != E; ++I) {
    getEqualValues(I->second, equalValues);
    uint32_t size = equalValues.size();
    
    if(size > 1)
      valueCountVector.push_back(std::make_pair(I->first, size));
    
    equalValues.clear();
  }
    
  return valueCountVector;
}
