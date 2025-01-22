import pulp

def make_positive(A):
    min_val = min(min(row) for row in A)
    if min_val <= 0:
        M = -min_val + 1
    else:
        M = 0
    A_pos = [[x+M for x in row] for row in A]
    return A_pos, M

def solve_p1(A):
    a,b=A[0][0],A[0][1]
    c,d=A[1][0],A[1][1]
    prob=pulp.LpProblem("P1",pulp.LpMaximize)
    x1=pulp.LpVariable('x1',lowBound=0)
    x2=pulp.LpVariable('x2',lowBound=0)
    vx=pulp.LpVariable('vx')
    prob+=vx
    prob+=x1+x2==1
    prob+=x1*a+x2*c>=vx
    prob+=x1*b+x2*d>=vx
    prob.solve(pulp.PULP_CBC_CMD(msg=0))
    return pulp.value(x1),pulp.value(x2),pulp.value(vx)

def solve_p2(A):
    a,b=A[0][0],A[0][1]
    c,d=A[1][0],A[1][1]
    prob=pulp.LpProblem("P2",pulp.LpMinimize)
    y1=pulp.LpVariable('y1',lowBound=0)
    y2=pulp.LpVariable('y2',lowBound=0)
    vy=pulp.LpVariable('vy')
    prob+=vy
    prob+=y1+y2==1
    prob+=y1*a+y2*b<=vy
    prob+=y1*c+y2*d<=vy
    prob.solve(pulp.PULP_CBC_CMD(msg=0))
    return pulp.value(y1),pulp.value(y2),pulp.value(vy)

A=[[3,1],[0,2]]
A, M = make_positive(A)
x1,x2,vx=solve_p1(A)
y1,y2,vy=solve_p2(A)
print("P1 mixed strategy:",x1,x2)
print("P2 mixed strategy:",y1,y2)
print("Game value P1:",vx)
print("Game value P2:",-vx)
