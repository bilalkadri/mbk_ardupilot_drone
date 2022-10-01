a=[0]*2*2
a[0]=1
a[1]=2
a[2]=3
a[3]=4
# a[4]=5

#a=[1,2,3]
print('a[0]={0},a[1],={1},a[2]={2}'.format(a[0],a[1],a[2]))
PossIr=[0]*5

number_of_membership_functions=5
i=0
# for i in range(number_of_membership_functions):
#     print(a[i])

def Fuzzification(r):

    if r<0:
        r=0
    elif r>1:
        r=1
    elif r>=0  or r<=1:
        r=r
    
    number_of_membership_functions=5

    count=1
    rc=[0]*15

    rc[0]=-0.25
    rc[1]=0
    rc[2]=0.25

    rc[3]=0
    rc[4]=0.25
    rc[5]=0.50

    rc[6]=0.25
    rc[7]=0.50
    rc[8]=0.75

    rc[9]=0.50
    rc[10]=0.75
    rc[11]=1.0

    rc[12]=0.75
    rc[13]=1.0
    rc[14]=1.25


    i=0
    for i in range(number_of_membership_functions):
        var1=function_U(r,rc[count-1],rc[count])
        var2=function_U(r,rc[count],rc[count+1])
        PossIr[i]=(r-rc[count-1])/(rc[count]-rc[count-1])*var1+(rc[count+1]-r)/(rc[count+1]-rc[count])*var2
        count=count+3
    

    if r<0:
        PossIr[0]=1
        PossIr[1:4]=0
    elif r>1:
        PossIr[0:3]=0
        PossIr[4]=1

    return PossIr 

def function_U(r,c1,c2):
              
    if r>=c1 and r<c2:
        x=1
    else:
        x=0
    return x



def Array_Passing(e):
    first_element=e[0]
    second_element=e[1]

    sum=first_element+second_element
    return sum

	
if __name__ == "__main__":
    
    
    # value=Fuzzification(0.675)
    # print(value)
    x=[1,2]
    result=Array_Passing(x)
    print(result)
    


