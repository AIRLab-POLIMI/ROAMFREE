import sys
import string
import re

def matchingBracket(str, pos):
	cnt = 1
	for m in range(pos+1, len(str)):
		if str[m] == '(':
			cnt = cnt + 1
		if str[m] == ')':
			cnt = cnt - 1
		if cnt == 0:
			return m
			
def removeHoldFunctions(s):
	ret = s
		
	while True:
		p = string.find(ret, 'Hold(')		
		if p == -1:
			break;		

		ep = matchingBracket(ret,p+4)		
		ret = ret[0:p]+ret[p+5:ep]+ret[ep+1:]
			
	return ret
	
def propersplit(s, sep):

	# split the string in meaningful places
	
	ret = list()

	cnt = 0 
	last = 0
	for m in range(0,len(s)):
		if s[m] == ',' and cnt == 0:
			ret.append(s[last:m])
			last = m+1
		if s[m] == '(':
			cnt = cnt+1
		if s[m] == ')':
			cnt = cnt-1

	ret.append(s[last:])
	return ret

	
def parseList(s):
	tmp = s
	
	ret = list()
	
	while True:
		p = string.find(tmp, 'List(')
		if p == -1:
			break;
		
		ep = matchingBracket(tmp,p+4)
		
		elem = parseList(tmp[p+5:ep])
		ret.append(elem)
		tmp = tmp[0:p]+tmp[ep+1:]
	
	if len(ret) > 0:
		return ret
	else:
		return propersplit(s,',')
		
def names(i):   
	return 'tmp{0}'.format(tmps[i.group(1)])

def fixMathFunctions(l):
	l = re.sub('Power\(E', 'std::pow(M_E', l)
	l = re.sub('Power\(', 'std::pow(', l)		
	l = re.sub('Abs\(', 'std::fabs(', l)		
	l = re.sub('ArcCos\(', 'std::acos(', l)
	l = re.sub('ArcSin\(', 'std::asin(', l)
	l = re.sub('ArcTan\(', 'std::atan2(', l)
	l = re.sub('Tan\(', 'std::tan(', l)
	l = re.sub('Sqrt\(', 'std::sqrt(', l)
	l = re.sub('Sin\(', 'std::sin(', l)
	l = re.sub('Cos\(', 'std::cos(', l)
	l = re.sub('Sec\(', '1.0/std::cos(', l)
	l = re.sub('Cot\(', '1.0/std::tan(', l)
	l = re.sub('Csc\(', '1.0/std::sin(', l)  
	
	#fresnel integrals (!!!)
	l = re.sub('FresnelC\(', 'ROAMmath::fresnel_c(', l)  
	l = re.sub('FresnelS\(', 'ROAMmath::fresnel_s(', l)  
	
	#pi
	l = re.sub('Pi', 'M_PI', l)  
	l = re.sub('mM_PItch', 'mPitch', l) 
	
	# we have to swap the two arguments of the ArcTan because of odd Mathematica conventions

	cnt = 0;
	while True:		
		
		occ = string.find(l,'atan2',cnt)

		if occ == -1:
			break

		start = occ+6		
		pt = occ+6			

		depth=1
		while depth>0:
			if l[pt] == '(':
				depth = depth + 1
			elif l[pt] == ')':
				depth = depth - 1
			elif l[pt] == ',':
				if depth == 1:
					coma = pt

			pt = pt + 1

		end = pt				

		l = l[:start]+l[coma+1:end-1]+','+l[start:coma]+')'+l[end:]
		
		cnt = occ + 5

	# we have to replace Sign(x) with (0.0 < x) - (x < 0)

	cnt = 0;
	while True:		
		
		occ = string.find(l,'Sign',cnt)

		if occ == -1:
			break

		start = occ+5		
		pt = occ+5			

		depth=1
		while depth>0:
			if l[pt] == '(':
				depth = depth + 1
			elif l[pt] == ')':
				depth = depth - 1

			pt = pt + 1

		end = pt				

		l = l[:occ]+'((0.0 < '+l[start:end-1]+') - ('+l[start:end-1]+' < 0.0))'+l[end:]
		
		cnt = occ + 4
		

	return l
  
def arrays(i):	
	if i.group(4) == None:
		sndindex =''
	else:
		sndindex = ',{0}+_OFF'.format(i.group(4))
		
	return '{0}({1}+_OFF{2})'.format(i.group(1), i.group(2),sndindex)


def reshapeTwoArgsFunction(C, str, op):
	
	start = 0	
	nC = ''
	while True:
		pos = C.find(str,start);		

		if pos == -1:
			nC = nC+C[start:]
			break

		nC = nC+C[start:pos]

		# scan until we find , 
		pos1 = pos+4
		k = pos1
		cnt = 0
		while True:
			if C[k] == '(':
				cnt = cnt + 1
			if C[k] == ')':
				cnt = cnt - 1

				if cnt == -1:
					pos3 = k
					break
			if C[k] == ',' and cnt == 0:
				pos2 = k

			k = k+1

		nC = nC+'('+reshapeTwoArgsFunction(C[pos1:pos2],str, op) +')'+op+'('+reshapeTwoArgsFunction(C[pos2+1:pos3], str, op)+')'

		start = pos3+1

	return nC
			
endname = string.find(sys.argv[1], '.');

if sys.argv[3]=='1':
	ext = 'm'
else:
	ext = 'cppready'

f = open(sys.argv[1], 'r')
g = open(sys.argv[1][:endname]+'.'+ext, 'w');

l = f.readline()

# remove Hold functions

tmp1 = removeHoldFunctions(l)

# remove bracket nesting all the expression

if tmp1[0] == '(':
	tmp1=tmp1[1:-1]

# find the important expression 

p = string.find(tmp1, 'List(')
ep = matchingBracket(tmp1,p+4)

temporaries = tmp1[0:p]+'\n'

expression = parseList(tmp1[p+5:ep])

# find temporaries

#ret = re.findall('Compile_\$\$(\d+)',temporaries) //vecchia versione di mathematica
ret = re.findall('Compile_\$(\d+)',temporaries)

# create dictionary for temporary variables

tmps = dict()
for k in ret:
	if not tmps.has_key(k):		
		tmps[k] = len(tmps)

# output temporaries definition

C = ''

tmpslist = propersplit(temporaries,',')
tmpslist = tmpslist[0:len(tmpslist)-1]

for t in tmpslist:
	C = C + 'double {0};\n'.format(t)


if type(expression[0]) == list:
	print "matrix", len(expression),'x',len(expression[0])
	
	for i in range(len(expression)):
		for j in range(len(expression[0])):
			C = C + '{0}({1}+_OFF,{2}+_OFF) = {3};\n'.format(sys.argv[2],i+1,j+1,expression[i][j])
else:
	print "vector", len(expression)
	
	for i in range(len(expression)):
		C = C + '{0}({1}+_OFF) = {2};\n'.format(sys.argv[2],i+1,expression[i])

# replace temporaries names

C = re.sub('Compile_\$(\d+)', names, C)

# replace indices with indices + _OFF

C = re.sub('([a-zA-Z_][a-zA-Z0-9_]*)\(([0-9]+)(,\s*([0-9])+)*\)', arrays, C)

# fix stupid mathematica function names

C = fixMathFunctions(C)

# fuck unneeded spaces

C = re.sub('[ ]+', ' ', C)

# now if we want matlab output we just need to cut away some bullshit

if sys.argv[3]=='1':
	C = re.sub('double ', '', C)
	C = re.sub('\+_OFF', '', C)
	C = re.sub('std::', '', C)

	# and we want pow(a,b) substituted with (a)^(b)
	C = reshapeTwoArgsFunction(C,'pow','^')
	
if sys.argv[4]=='1':
	# we are working with sparse assignments, strip away everything which is = 0;	
	C = re.sub('[^\n]+ = 0;\s*','',C)
	

print C

g.write(C)
