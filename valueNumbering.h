using namespace llvm;

//----------------------------------------------------------------------===
 //                         ValueTable Class

//----------------------------------------------------------------------===
 
 /// This class holds the mapping between values and value numbers.  It is used
 /// as an efficient mechanism to determine the expression-wise equivalence of
 /// two values.

struct Expression {
  enum ExpressionOpcode { ADD, FADD, SUB, FSUB, MUL, FMUL,
                          UDIV, SDIV, FDIV, UREM, SREM,
                          FREM, SHL, LSHR, ASHR, AND, OR, XOR, ICMPEQ,
                          ICMPNE, ICMPUGT, ICMPUGE, ICMPULT, ICMPULE,
                          ICMPSGT, ICMPSGE, ICMPSLT, ICMPSLE, FCMPOEQ,
                          FCMPOGT, FCMPOGE, FCMPOLT, FCMPOLE, FCMPONE,
                          FCMPORD, FCMPUNO, FCMPUEQ, FCMPUGT, FCMPUGE,
                          FCMPULT, FCMPULE, FCMPUNE, EXTRACT, INSERT,
                          SHUFFLE, SELECT, TRUNC, ZEXT, SEXT, FPTOUI,
                          FPTOSI, UITOFP, SITOFP, FPTRUNC, FPEXT,
                          PTRTOINT, INTTOPTR, BITCAST, GEP, CALL, ANT,
                          INSERTVALUE, EXTRACTVALUE, EMPTY, TOMBSTONE };
 
  ExpressionOpcode opcode;
  const Type* type;
  SmallVector<uint32_t, 4> varargs;
 
  Expression() { }
  Expression(ExpressionOpcode o) : opcode(o) { }
 
  bool operator==(const Expression &other) const {
    if (opcode != other.opcode)
      return false;
    else if (opcode == EMPTY || opcode == TOMBSTONE)
      return true;
    else if (type != other.type)
      return false;
    else {
      if (varargs.size() != other.varargs.size())
        return false;
 
      for (size_t i = 0; i < varargs.size(); ++i)
        if (varargs[i] != other.varargs[i])
          return false;
 
      return true;
    }
  }
 
  bool operator!=(const Expression &other) const {
    return !(*this == other);
  }
};
 
class ValueTable {
  private:
    DenseMap<Value*, uint32_t> valueNumbering;
    DenseMap<Expression, uint32_t> expressionNumbering;
    DenseMap<Value*, uint32_t> constantsNumbering;
    DenseMap<uint32_t, Value*> leaderBoard;
 
    uint32_t nextValueNumber;
    uint32_t maxValueNumber;
 
    Expression::ExpressionOpcode getOpcode(BinaryOperator* BO);
    Expression::ExpressionOpcode getOpcode(CmpInst* C);
    Expression::ExpressionOpcode getOpcode(CastInst* C);
    Expression create_expression(BinaryOperator* BO);
    Expression create_expression(CmpInst* C);
    Expression create_expression(ShuffleVectorInst* V);
    Expression create_expression(ExtractElementInst* C);
    Expression create_expression(InsertElementInst* V);
    Expression create_expression(SelectInst* V);
    Expression create_expression(CastInst* C);
    Expression create_expression(GetElementPtrInst* G);
    Expression create_expression(CallInst* C);
    Expression create_expression(Constant* C);
    Expression create_expression(ExtractValueInst* C);
    Expression create_expression(InsertValueInst* C);
     
    // We need to access private members valueNumbering and leaderBoard from RPO
    friend class RPO;

  public:
    ValueTable() : nextValueNumber(1), maxValueNumber(1) { }
    uint32_t computeNumber(Value *V);
    uint32_t lookup(Value *V);
    void add(Value *V, uint32_t num);
    void clear();
    void clearExpressions();
    void erase(Value *v);
    unsigned size();
    void verifyRemoved(const Value *) const;
    uint32_t isSimplePHI(PHINode* PN);
};


namespace llvm{
 template <> struct DenseMapInfo<Expression> {
   static inline Expression getEmptyKey() {
     return Expression(Expression::EMPTY);
   }
 
   static inline Expression getTombstoneKey() {
     return Expression(Expression::TOMBSTONE);
   }
 
   static unsigned getHashValue(const Expression e) {
     unsigned hash = e.opcode;
 
     hash = ((unsigned)((uintptr_t)e.type >> 4) ^
             (unsigned)((uintptr_t)e.type >> 9));
 
     for (SmallVector<uint32_t, 4>::const_iterator I = e.varargs.begin(),
          E = e.varargs.end(); I != E; ++I)
       hash = *I + hash * 37;
 
     return hash;
   }
   static bool isEqual(const Expression &LHS, const Expression &RHS) {
     return LHS == RHS;
   }
 };
 template <>
 struct isPodLike<Expression> { static const bool value = true; };
}

//----------------------------------------------------------------------===
 //                         RPO Class

//----------------------------------------------------------------------===

class RPO {

  private :
    Function& F;
    LoopInfo* LI;
    ValueTable VT;
    DenseMap<uint32_t, uint32_t> VNtoBVPos;
    std::vector<Instruction*> deadList;
    
  public:
    RPO(Function &f, LoopInfo* li) : F(f), LI(li) {}
    void performVN();             
    
    // this function handles the special case when the value number of op1 = op2
    // and the operator is either AND, OR, CMP::EQ, CMP::NE
    void handleSpecialCases();

    // length of the bit-vector for DFA is the number of repeated values. This
    // function does a hash from Value Number to position in Bit Vector
    void calculateBitVectorPosition();
    uint32_t getBitVectorPosition(Value* V);
    uint32_t getBitVectorPosition(uint32_t vn);
    uint32_t getVNFromBVPos(uint32_t bvPos);

    // print all instructions with their corresponding VN
    void print();
    void cleanUp();

    // functions to modify valueNumbering table
    void eraseValue(Value* V);
    void addValue(Value* V, uint32_t vn);
    
    // -- Define all interface functions below --
    
    uint32_t getNumberForValue(Value *V);

    // get all Values with VN same as that of input. The resulting vector
    // includes the incoming Value as well
    void getEqualValues(Value* V, SmallVectorImpl<Value*> &equalValues);
    void getEqualValues(uint32_t VN, SmallVectorImpl<Value*> &equalValues);

    // get the Leader(Producer) of a Value. Note that this may return NULL for
    // some Values. More specifically, if the expression doesn't call
    // create_expression in ValueTable::computeNumber() then it is never added
    // to the leaderBoard, and hence this function returns NULL. Examples
    // include branch and call instructions.
    // Also note that the Value returned by this function does NOT necessarily
    // dominate the input Value!
    Value* getLeader(Value *V);
    Value* getLeader(uint32_t vn);

    // List of all leaders
    std::vector<Value*> getAllLeaders();

    // this function returns a vector of pair. A pair is for a Value which 
    // occurs more than once in the function. The count of this vector gives 
    // the smallest size of the bitvector needed for the data flow equations. 
    // Return format <VN, Count>
    std::vector<std::pair<uint32_t, uint32_t> > getRepeatedValues();
};
