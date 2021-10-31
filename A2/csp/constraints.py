from csp import Constraint, Variable
import math
import util

class TableConstraint(Constraint):
    '''General type of constraint that can be use to implement any type of
       constraint. But might require a lot of space to do so.

       A table constraint explicitly stores the set of satisfying
       tuples of assignments.'''

    def __init__(self, name, scope, satisfyingAssignments):
        '''Init by specifying a name and a set variables the constraint is over.
           Along with a list of satisfying assignments.
           Each satisfying assignment is itself a list, of length equal to
           the number of variables in the constraints scope.
           If sa is a single satisfying assignment, e.g, sa=satisfyingAssignments[0]
           then sa[i] is the value that will be assigned to the variable scope[i].


           Example, say you want to specify a constraint alldiff(A,B,C,D) for
           three variables A, B, C each with domain [1,2,3,4]
           Then you would create this constraint using the call
           c = TableConstraint('example', [A,B,C,D],
                               [[1, 2, 3, 4], [1, 2, 4, 3], [1, 3, 2, 4],
                                [1, 3, 4, 2], [1, 4, 2, 3], [1, 4, 3, 2],
                                [2, 1, 3, 4], [2, 1, 4, 3], [2, 3, 1, 4],
                                [2, 3, 4, 1], [2, 4, 1, 3], [2, 4, 3, 1],
                                [3, 1, 2, 4], [3, 1, 4, 2], [3, 2, 1, 4],
                                [3, 2, 4, 1], [3, 4, 1, 2], [3, 4, 2, 1],
                                [4, 1, 2, 3], [4, 1, 3, 2], [4, 2, 1, 3],
                                [4, 2, 3, 1], [4, 3, 1, 2], [4, 3, 2, 1]])
          as these are the only assignments to A,B,C respectively that
          satisfy alldiff(A,B,C,D)
        '''

        Constraint.__init__(self,name, scope)
        self._name = "TableCnstr_" + name
        self.satAssignments = satisfyingAssignments

    def check(self):
        '''check if current variable assignments are in the satisfying set'''
        assignments = []
        for v in self.scope():
            if v.isAssigned():
                assignments.append(v.getValue())
            else:
                return True
        return assignments in self.satAssignments

    def hasSupport(self, var,val):
        '''check if var=val has an extension to an assignment of all variables in
           constraint's scope that satisfies the constraint. Important only to
           examine values in the variable's current domain as possible extensions'''
        if var not in self.scope():
            return True   #var=val has support on any constraint it does not participate in
        vindex = self.scope().index(var)
        found = False
        for assignment in self.satAssignments:
            if assignment[vindex] != val:
                continue   #this assignment can't work it doesn't make var=val
            found = True   #Otherwise it has potential. Assume found until shown otherwise
            for i, v in enumerate(self.scope()):
                if i != vindex and not v.inCurDomain(assignment[i]):
                    found = False  #Bummer...this assignment didn't work it assigns
                    break          #a value to v that is not in v's curDomain
                                   #note we skip checking if val in in var's curDomain
            if found:     #if found still true the assigment worked. We can stop
                break
        return found     #either way found has the right truth value


class QueensConstraint(Constraint):
    '''Queens constraint between queen in row i and row j'''
    def __init__(self, name, qi, qj, i, j):
        scope = [qi, qj]
        Constraint.__init__(self,name, scope)
        self._name = "QueenCnstr_" + name
        self.i = i
        self.j = j

    def check(self):
        qi = self.scope()[0]
        qj = self.scope()[1]
        if not qi.isAssigned() or not qj.isAssigned():
            return True
        return self.queensCheck(qi.getValue(),qj.getValue())

    def queensCheck(self, vali, valj):
        diag = abs(vali - valj) == abs(self.i - self.j)
        return not diag and vali != valj
    def hasSupport(self, var, val):
        '''check if var=val has an extension to an assignment of the
           other variable in the constraint that satisfies the constraint'''
        #hasSupport for this constraint is easier as we only have one
        #other variable in the constraint.
        if var not in self.scope():
            return True   #var=val has support on any constraint it does not participate in
        otherVar = self.scope()[0]
        if otherVar == var:
            otherVar = self.scope()[1]
        for otherVal in otherVar.curDomain():
            if self.queensCheck(val, otherVal):
                return True
        return False

class QueensTableConstraint(TableConstraint):
    '''Queens constraint between queen in row i and row j, but
       using a table constraint instead. That is, you
       have to create and add the satisfying tuples.

       Since we inherit from TableConstraint, we can
       call TableConstraint.__init__(self,...)
       to set up the constraint.

       Then we get hasSupport and check automatically from
       TableConstraint
    '''
    #your implementation for Question 1 goes
    #inside of this class body. You must not change
    #the existing function signatures.
    def __init__(self, name, qi, qj, i, j):
        satisfyingAssignments = []
        diag_val = abs(i - j)
        for qi_val in qi.domain():
            for qj_val in qj.domain():
                if qj_val != qi_val - diag_val and qj_val != qi_val and qj_val != qi_val + diag_val:
                    satisfyingAssignments.append([qi_val, qj_val])
        TableConstraint.__init__(self, name, [qi, qj], satisfyingAssignments)

class NeqConstraint(Constraint):
    '''Neq constraint between two variables'''
    def __init__(self, name, scope):
        if len(scope) != 2:
            print("Error Neq Constraints are only between two variables")
        Constraint.__init__(self,name, scope)
        self._name = "NeqCnstr_" + name

    def check(self):
        v0 = self.scope()[0]
        v1 = self.scope()[1]
        if not v0.isAssigned() or not v1.isAssigned():
            return True
        return v0.getValue() != v1.getValue()

    def hasSupport(self, var, val):
        '''check if var=val has an extension to an assignment of the
           other variable in the constraint that satisfies the constraint'''
        #hasSupport for this constraint is easier as we only have one
        #other variable in the constraint.
        if var not in self.scope():
            return True   #var=val has support on any constraint it does not participate in
        otherVar = self.scope()[0]
        if otherVar == var:
            otherVar = self.scope()[1]
        for otherVal in otherVar.curDomain():
            if val != otherVal:
                return True
        return False

class AllDiffConstraint(Constraint):
    '''All diff constraint between a set of variables
       If you are curious as to how to more efficiently perform GAC on
       an AllDiff see
       http://www.constraint-programming.com/people/regin/papers/alldiff.pdf'''
    def __init__(self, name, scope):
        Constraint.__init__(self,name, scope)
        self._name = "AllDiff_" + name

    def check(self):
        assignments = []
        for v in self.scope():
            if v.isAssigned():
                assignments.append(v.getValue())
            else:
                return True
        return len(set(assignments)) == len(assignments)

    def hasSupport(self, var, val):
        '''check if var=val has an extension to an assignment of the
           other variable in the constraint that satisfies the constraint'''
        if var not in self.scope():
            return True   #var=val has support on any constraint it does not participate in

        #since the contraint has many variables use the helper function 'findvals'
        #for that we need two test functions
        #1. for testing complete assignments to the constraint's scope
        #   return True if and only if the complete assignment satisfies the constraint
        #2. for testing partial assignments to see if they could possibly work.
        #   return False if the partial assignment cannot be extended to a satisfying complete
        #   assignment
        #
        #Function #2 is only needed for efficiency (sometimes don't have one)
        #  if it isn't supplied findvals will use a function that never returns False
        #
        #For alldiff, we do have both functions! And they are the same!
        #We just check if the assignments are all to different values. If not return False
        def valsNotEqual(l):
            '''tests a list of assignments which are pairs (var,val)
               to see if they can satisfy the all diff'''
            vals = [val for (var, val) in l]
            return len(set(vals)) == len(vals)
        varsToAssign = self.scope()
        varsToAssign.remove(var)
        x = findvals(varsToAssign, [(var, val)], valsNotEqual, valsNotEqual)
        return x


def findvals(remainingVars, assignment, finalTestfn, partialTestfn=lambda x: True):
    '''Helper function for finding an assignment to the variables of a constraint
       that together with var=val satisfy the constraint. That is, this
       function looks for a supporing tuple.

       findvals uses recursion to build up a complete assignment, one value
       from every variable's current domain, along with var=val.

       It tries all ways of constructing such an assignment (using
       a recursive depth-first search).

       If partialTestfn is supplied, it will use this function to test
       all partial assignments---if the function returns False
       it will terminate trying to grow that assignment.

       It will test all full assignments to "allVars" using finalTestfn
       returning once it finds a full assignment that passes this test.

       returns True if it finds a suitable full assignment, False if none
       exist. (yes we are using an algorithm that is exactly like backtracking!)'''

    # print("==>findvars([",)
    # for v in remainingVars: print(v.name(), " ", end='')
    # print("], [",)
    # for x,y in assignment: print("({}={}) ".format(x.name(),y), end='')
    # print("")

    #sort the variables call the internal version with the variables sorted
    remainingVars.sort(reverse=True, key=lambda v: v.curDomainSize())
    return findvals_(remainingVars, assignment, finalTestfn, partialTestfn)

def findvals_(remainingVars, assignment, finalTestfn, partialTestfn):
    '''findvals_ internal function with remainingVars sorted by the size of
       their current domain'''
    if len(remainingVars) == 0:
        return finalTestfn(assignment)
    var = remainingVars.pop()
    for val in var.curDomain():
        assignment.append((var, val))
        if partialTestfn(assignment):
            if findvals_(remainingVars, assignment, finalTestfn, partialTestfn):
                return True
        assignment.pop()   #(var,val) didn't work since we didn't do the return
    remainingVars.append(var)
    return False


class NValuesConstraint(Constraint):
    '''NValues constraint over a set of variables.  Among the variables in
       the constraint's scope the number that have been assigned
       values in the set 'required_values' is in the range
       [lower_bound, upper_bound] (lower_bound <= #of variables
       assigned 'required_value' <= upper_bound)

       For example, if we have 4 variables V1, V2, V3, V4, each with
       domain [1, 2, 3, 4], then the call
       NValuesConstraint('test_nvalues', [V1, V2, V3, V4], [3,2], 2,
       3) will only be satisfied by assignments such that at least 2
       the V1, V2, V3, V4 are assigned the value 3 or 2, and at most 3
       of them have been assigned the value 3 or 2.

    '''

    def __init__(self, name, scope, required_values, lower_bound, upper_bound):
        Constraint.__init__(self,name, scope)
        self._name = "NValues_" + name
        self._required = required_values
        self._lb = lower_bound
        self._ub = upper_bound

    def check(self):
        satisfy_num = 0
        for v in self.scope():
            if v.isAssigned() and v.getValue() in self._required:
                satisfy_num += 1
            else:
                return True
        return satisfy_num >= self._lb and satisfy_num <= self._ub

    def hasSupport(self, var, val):
        '''check if var=val has an extension to an assignment of the
           other variable in the constraint that satisfies the constraint

           HINT: check the implementation of AllDiffConstraint.hasSupport
                 a similar approach is applicable here (but of course
                 there are other ways as well)
        '''
        if var not in self.scope():
            return True   #var=val has support on any constraint it does not participate in
        satisfy_num = 0
        if val in self._required:
            satisfy_num += 1
        varsToAssign = self.scope()
        varsToAssign.remove(var)
        while len(varsToAssign) != 0:
            varToAssign = varsToAssign.pop()
            for var_val in varToAssign.curDomain():
                if var_val in self._required:
                    satisfy_num += 1
                    break
        return satisfy_num >= self._lb and satisfy_num <= self._ub

class ValidInitFlightsConstraint(Constraint):
    def __init__(self, name, scope, valid_init_flights):
        Constraint.__init__(self,name, scope)
        self._name = "C2_ValidInitFlights_" + name
        self._valid_init_flights = valid_init_flights

    def check(self):
        first_none_idle_flight = None
        for v in self.scope():
            if v.isAssigned() and v.getValue() != "idle":
                first_none_idle_flight = v.getValue()
        return first_none_idle_flight == None or first_none_idle_flight in self._valid_init_flights

    def hasSupport(self, var, val):
        if var not in self.scope():
            return True
        first_var = self.scope()[0]
        # Check if the first variable is a valid initial flight
        if first_var.isAssigned():
            if first_var.getValue() in self._valid_init_flights:
                return True
            if first_var.getValue() != "idle":
                return False
        if first_var == var:
            if val in self._valid_init_flights:
                return True
            if val != "idle":
                return False
        for cur_val in first_var.curDomain():
            if cur_val in self._valid_init_flights:
                return True
        # Special case: all idle, i.e. no flights taken is also valid
        all_idle = True
        for cur_var in self.scope():
            if "idle" not in cur_var.curDomain():
                all_idle = False
            if cur_var == var and val != "idle":
                all_idle = False
        return all_idle

class ValidFlightsSeqConstraint(Constraint):
    def __init__(self, name, scope, valid_flights_seq):
        Constraint.__init__(self,name, scope)
        self._name = "C3_ValidFlightsSeq_" + name
        self._valid_flights_seq = valid_flights_seq

    def check(self):
        flight_list = list()
        for var in self.scope():
            if var.isAssigned() and var.getValue() != "idle":
                flight_list.append(var.getValue())
        for flight_index in range(0, len(flight_list) - 1):
            if (flight_list[flight_index], flight_list[flight_index + 1]) not in self._valid_flights_seq:
                return False
        return True

    def hasSupport(self, var, val):
        if var not in self.scope():
            return True
        valid_seq = [False] * (len(self.scope()) - 1)
        # Check all possible sequence, return true if all pairs have one
        for var_index in range(0, len(self.scope()) - 1):
            for first_val in self.scope()[var_index].curDomain():
                for second_val in self.scope()[var_index + 1].curDomain():
                    if var == self.scope()[var_index]:
                        first_val = val
                    if var == self.scope()[var_index + 1]:
                        second_val = val
                    if (first_val, second_val) in self._valid_flights_seq \
                        or (first_val, second_val) == ("idle", "idle") or second_val == "idle":
                        valid_seq[var_index] = True
        for seq in valid_seq:
            if seq == False:
                return False
        return True

class ValidMaintenanceConstraint(Constraint):
    def __init__(self, name, scope, maintenance_flights, min_maintenance_frequency):
        Constraint.__init__(self,name, scope)
        self._name = "C4_ValidMaintenance_" + name
        self._maintenance_flights = maintenance_flights
        self._min_maintenance_frequency = min_maintenance_frequency

    def check(self):
        flights_count = 0
        for cur_var in self.scope():
            if cur_var.isAssigned() and cur_var.getValue() != "idle" and cur_var.getValue() not in self._maintenance_flights:
                flights_count += 1
        if flights_count < self._min_maintenance_frequency:
            return True
        for var_index in range(0, len(self.scope()) - self._min_maintenance_frequency + 1):
            flights_count = 0
            for cur_window_index in range(0, self._min_maintenance_frequency):
                cur_var = self.scope()[var_index + cur_window_index]
                if cur_var.isAssigned() and cur_var.getValue() != "idle" and cur_var.getValue() not in self._maintenance_flights:
                    flights_count += 1
            if flights_count >= self._min_maintenance_frequency:
                return False
        return True

    def hasSupport(self, var, val):
        if var not in self.scope():
            return True
        if len(self.scope()) <= self._min_maintenance_frequency:
            return True
        # Track accumulated flight with no maintenance
        # Clear when encouter a maintenance stop
        flights_count = 0
        for cur_var in self.scope():
            if cur_var.isAssigned():
                if cur_var.getValue() in self._maintenance_flights:
                    flights_count = 0
                elif cur_var.getValue() != "idle" and cur_var.getValue() not in self._maintenance_flights:
                    flights_count += 1
            elif cur_var == var:
                if val in self._maintenance_flights:
                    flights_count = 0
                if val != "idle" and val not in self._maintenance_flights:
                    flights_count += 1
            elif not cur_var.isAssigned():
                for cur_var_val in cur_var.curDomain():
                    if cur_var_val in self._maintenance_flights:
                        flights_count = 0
            if flights_count >= self._min_maintenance_frequency:
                return False
        return True

class ValidAssignmentConstraint(Constraint):
    def __init__(self, name, scope, flight):
        Constraint.__init__(self,name, scope)
        self._name = "C5_ValidAssignment_" + name
        self._flight = flight

    def check(self):
        flight_count = 0
        for cur_var in self.scope():
            if cur_var.isAssigned() and cur_var.getValue() != self._flight:
                flight_count += 1
        return flight_count == 1

    def hasSupport(self, var, val):
        flight_count = 0
        for cur_var in self.scope():
            if cur_var.isAssigned() and cur_var.getValue() == self._flight:
                flight_count += 1
        if val == self._flight:
            flight_count += 1
        if flight_count == 1:
            return True
        if flight_count > 1:
            return False
        for cur_var in self.scope():
            if not cur_var.isAssigned() and cur_var != var:
                for cur_var_val in cur_var.curDomain():
                    if cur_var_val == self._flight:
                        return True
        return False
