import copy


class Solver:
    # Initializing the input CNF and converting the sets into lists
    def __init__(self, cnf: object) -> object:
        self.cnf = copy.deepcopy(cnf)
        for i in range(len(cnf)):
            cnf[i] = list(cnf[i])
        self.input_cnf = cnf

    # the wraper function for dpll fucntion
    def solve(self):
        res = self.dpll(self.input_cnf)
        return res

    # function which solves fand equation
    def dpll(self, cnf):
        if [] in cnf:
            return False
        elif not cnf:
            return True
        else:
            # unit propagation
            for clause in cnf:
                if len(clause) == 1:
                    item = clause[0]
                    index = 0
                    while -1 < index < len(cnf):
                        if item in cnf[index]:
                            cnf.pop(index)
                            index -= 1
                        elif -item in cnf[index]:
                            cnf[index].remove(-item)
                        index += 1
                    return self.dpll(copy.deepcopy(cnf))
            # pure literal elimination and assigning true or false to all variable to check for solvability
            for i in cnf:
                for j in i:
                    return self.dpll(self.reduce(copy.deepcopy(cnf), j)) or self.dpll(self.reduce(copy.deepcopy(cnf), -j))

    # the reduce function to check an equation but assigning the literal give as true in the input CNF
    # and returning the resulting CNF back to the calling function.
    def reduce(self, cnf_up, literal):
        index = 0
        while -1 < index < len(cnf_up):
            if literal in cnf_up[index]:
                cnf_up.pop(index)
                index -= 1
            elif -literal in cnf_up[index]:
                cnf_up[index].remove(-literal)
            index += 1
        return cnf_up


