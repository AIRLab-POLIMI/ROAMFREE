#!/usr/bin/env wolframscript
(* ::Package:: *)

Clear["Global`*"]



parentPath = $InputFileName /. "" :> NotebookFileName[]
parentDir = DirectoryName @ parentPath
Import[parentDir<>"SimplifyFunctions.m"]
Import[parentDir<>"QuaternionAlgebra.m"]


(* ::Title:: *)
(*Useful Functions*)


L2[x_]:=Sqrt[Sum[x[[j]]^2,{j,1,Length[x]}]];
VecNorm[x_]:=Table[x[[i]]/L2[x],{i,1,Length[x]}];


SkewMatrix4[v_]:={{0,-v[[1]],-v[[2]],-v[[3]]},{v[[1]],0,v[[3]],-v[[2]]},{v[[2]],-v[[3]],0,v[[1]]},{v[[3]],v[[2]],-v[[1]],0}}/2;
SkewMatrix[v_]:={{0,-v[[3]],v[[2]]},{v[[3]],0,-v[[1]]},{-v[[2]],v[[1]],0}};


SubVec[exp_,v1_,v2_]:=exp/.Table[v1[[i]]->v2[[i]],{i,1,Dimensions[v1,1][[1]]}];
VecToZero[exp_,v_]:=exp/.Table[v[[i]]->0,{i,1,Dimensions[v,1][[1]]}];


ToGoodC[exp_]:=Module[{oexp}, oexp=Experimental`OptimizeExpression[exp];
If[Dimensions [StringPosition[ToString[InputForm[oexp]],"Compile"],1][[1]]>0,Block[{ locals, code},{locals,code}=ReleaseHold[(Hold@@oexp)/.Verbatim[Block][vars_,seq_]:>{vars,Hold[seq]}];ToString[CForm[code]]]
, ToString[CForm[exp]]]]
MyStringWrite[str_,file_]:=Module[{stream},stream=OpenWrite[file];WriteString[stream,str];Close[stream];]


(* ::Title:: *)
(*Beginning of stuff*)


(* ::Section:: *)
(*State*)


X=Table[x[i],{i,1,39}];
POSE=X[[{1,2,3}]];
Q=X[[{4,5,6,7}]];
V=X[[{8,9,10}]];
W=X[[{11,12,13}]];
A=X[[{14,15,16}]];
Alpha=X[[{17,18,19}]];
QWSprev = X[[30;;33]];
VSprev = X[[34;;36]];
WSprev = X[[37;;39]];


(* ::Section:: *)
(*Noises and Inputs*)


Z=Table[Abs[z[i]],{i,1,4}];
NOISES=Table[eta[i],{i,1,6}];
ZNoise = Z + Table[NOISES[[i]],{i,1,4}];


(* ::PageBreak:: *)
(**)


(* ::Section:: *)
(*Parameters*)


(* Wind planar in world coordinates *)
WINDXY = {windxy[1],windxy[2]};
(* Wind vertical in world coordinates *)
WINDZ = {windz[1]};
(* Wind in world coordinates *)
WIND={windxy[1],windxy[2],windz[1]}; 

(* Linear drag coefficients *)
LDRAG = {ldrag[1],ldrag[2],ldrag[3]};
(* Quadratic drag coefficients *)
QDRAG = {qdrag[1],qdrag[2],qdrag[3]};
(* Rotor drag coefficients *)
RDRAG={rdrag[1],rdrag[2],rdrag[3]};

(* Lambda parameter(s) *)
LAMBDA = {lambda[1]};
(* Mu parameter(s) *)
MU = {mu[1],mu[2]};

(* Mechanical properties of the rotors *)
MC = {mc[1], mc[2]};
b = mc[1];
d = mc[2];

(* overall copter properties *)
CP={cp[1],cp[2],cp[3]};
m=cp[1];
LX = cp[2];
LY = cp[3];

(* Inertia *)
IBD={ibd[1],ibd[2],ibd[3]};(* diagonal terms *)
IBOD={ibod[1],ibod[2],ibod[3]};(* off diagonal terms*)

Ib = {{Exp[ibd[1]],ibod[1],ibod[2]},{ibod[1],Exp[ibd[2]],ibod[3]},{ibod[2],ibod[3],Exp[ibd[3]]}};
Ibinv=Inverse[Ib]//FullSimplify;

GRAVITY = {0.0,0.0, -gravity[1]};


(* ::Section:: *)
(*Equations*)


(* ::Subsection:: *)
(*Forces*)


(* airspeed *)
AS=VSprev-QuatToRot[QuatInv[QWSprev]].WIND; 

(* Sum of rpms *)
SumRPM = Sum[ZNoise[[i]],{i,1,4}];

(* Thrust force *)
FT = {0,0,b Sum[ZNoise[[i]]^2,{i,1,4}]};

(* Rotor drag force - traditional *)
FR= SumRPM lambda[1]{AS[[1]],AS[[2]],0};

(* Rotor drag force - non-traditional *)
FRR =  SumRPM RDRAG;

(* Linear drag model *)
FDL = dragmodel[1] {ldrag[1]AS[[1]],ldrag[2]AS[[2]],ldrag[3]AS[[3]]};

(* Quadratic drag model *)
FDQ = dragmodel[2]Table[Exp[QDRAG[[i]]]  AS[[i]]^2 Sign[AS[[i]]],{i,1,3}];


(* ::Subsection:: *)
(*Moments*)


(* moments *)
MM={
LX b (-ZNoise[[1]]^2+ZNoise[[2]]^2+ZNoise[[3]]^2-ZNoise[[4]]^2), 
LY b (-ZNoise[[1]]^2+ZNoise[[2]]^2-ZNoise[[3]]^2+ZNoise[[4]]^2),
d (-ZNoise[[1]]^2-ZNoise[[2]]^2+ZNoise[[3]]^2+ZNoise[[4]]^2)};

(* Rolling moment 1 *)
kb = {0,0,1};
RM1 = SumRPM mu[1] {AS[[2]],-AS[[1]],0.0};

(* Rolling moment 2 *)
OmegaPerp ={WSprev[[1]],WSprev[[2]],0.0};
RM2 = -SumRPM mu[2] OmegaPerp;

(* Rolling moment 3 *)
l2 = LX*LX + LY*LY;
RM3 = {0,0,-lambda[1] WSprev[[3]] SumRPM l2};


(* ::Subsection:: *)
(*Euler equations*)


(* ::Subsubsection:: *)
(*Linear acceleration*)


forceBfPrev = (FT - FR+ FRR - FDL - FDQ )/m;
forceBf =  QuatToRot[QuatInv[Q]].QuatToRot[QWSprev].forceBfPrev;
AhatLong= forceBf+QuatToRot[QuatInv[Q]].GRAVITY+ {eta[5],eta[6],0};
Ahat = AhatLong;


(* ::Subsubsection:: *)
(*Angular acceleration*)


momentBfPrev = Ibinv.(MM+RM1+RM2+RM3 - SkewMatrix[WSprev].(Ib.WSprev));
AlphaHatLong =  QuatToRot[QuatInv[Q]].QuatToRot[QWSprev].momentBfPrev;
AlphaHat = AlphaHatLong;


(* ::PageBreak:: *)
(**)


(* ::Subsection:: *)
(*Errors*)


ERR = Flatten[{A-Ahat,Alpha-AlphaHat}];


(* ::Section:: *)
(*Output*)


SetDirectory[parentDir];
Reg={Sign'[_]->0,Abs'[x_]->Sign[x]};


(* ::Subsection:: *)
(*Jacobians*)


(* ::Subsubsection:: *)
(*State*)


Err = VecToZero[ERR,NOISES];
JErrNoises=VecToZero[D[ERR,{NOISES}]/.Reg,NOISES];
JErrQ=VecToZero[D[ERR,{Q}]/.Reg,NOISES];
JErrV=VecToZero[D[ERR,{V}]/.Reg,NOISES];
JErrW=VecToZero[D[ERR,{W}]/.Reg,NOISES];
JErrA=VecToZero[D[ERR,{A}]/.Reg,NOISES];
JErrAlpha=VecToZero[D[ERR,{Alpha}]/.Reg,NOISES];
JErrQprev=VecToZero[D[ERR,{QWSprev}]/.Reg,NOISES];
JErrVprev=VecToZero[D[ERR,{VSprev}]/.Reg,NOISES];
JErrWprev=VecToZero[D[ERR,{WSprev}]/.Reg,NOISES];


(* ::Subsubsection:: *)
(*Parameters*)


JErrWINDXY=VecToZero[D[ERR,{WINDXY}]/.Reg,NOISES];
JErrWINDZ=VecToZero[D[ERR,{WINDZ}]/.Reg,NOISES];
JErrLDRAG=VecToZero[D[ERR,{LDRAG}]/.Reg,NOISES];
JErrQDRAG=VecToZero[D[ERR,{QDRAG}]/.Reg,NOISES];
JErrRDRAG=VecToZero[D[ERR,{RDRAG}]/.Reg,NOISES];
JErrLAMBDA=VecToZero[D[ERR,{LAMBDA}]/.Reg,NOISES];
JErrMU=VecToZero[D[ERR,{MU}]/.Reg,NOISES];
JErrMC=VecToZero[D[ERR,{MC}]/.Reg,NOISES];
JErrCP=VecToZero[D[ERR,{CP}]/.Reg,NOISES];
JErrIBD=VecToZero[D[ERR,{IBD}]/.Reg,NOISES];
JErrIBOD=VecToZero[D[ERR,{IBOD}]/.Reg,NOISES];


(* ::Subsubsection:: *)
(*2nd order derivatives*)


AOut = VecToZero[AhatLong,NOISES];
AlphaOut = VecToZero[AlphaHatLong,NOISES];


(* ::Subsection:: *)
(*Mthout files*)


Run["rm *.cppready"];


(* ::Subsubsection:: *)
(*State*)


MyStringWrite[ToGoodC[Err],"QuadDynamicModel_Err.mthout"];
MyStringWrite[ToGoodC[JErrNoises],"QuadDynamicModel_JErrNoises.mthout"];
MyStringWrite[ToGoodC[JErrQ],"QuadDynamicModel_JErrQ.mthout"];
MyStringWrite[ToGoodC[JErrV],"QuadDynamicModel_JErrV.mthout"];
MyStringWrite[ToGoodC[JErrW],"QuadDynamicModel_JErrW.mthout"];
MyStringWrite[ToGoodC[JErrA],"QuadDynamicModel_JErrA.mthout"];
MyStringWrite[ToGoodC[JErrAlpha],"QuadDynamicModel_JErrAlpha.mthout"];
MyStringWrite[ToGoodC[JErrQprev],"QuadDynamicModel_JErrQprev.mthout"];
MyStringWrite[ToGoodC[JErrVprev],"QuadDynamicModel_JErrVprev.mthout"];
MyStringWrite[ToGoodC[JErrWprev],"QuadDynamicModel_JErrWprev.mthout"];


(* ::Subsubsection:: *)
(*Parameters*)


MyStringWrite[ToGoodC[JErrWINDXY],"QuadDynamicModel_JErrWINDXY.mthout"];
MyStringWrite[ToGoodC[JErrWINDZ],"QuadDynamicModel_JErrWINDZ.mthout"];
MyStringWrite[ToGoodC[JErrLDRAG],"QuadDynamicModel_JErrLDRAG.mthout"];
MyStringWrite[ToGoodC[JErrQDRAG],"QuadDynamicModel_JErrQDRAG.mthout"];
MyStringWrite[ToGoodC[JErrRDRAG],"QuadDynamicModel_JErrRDRAG.mthout"];
MyStringWrite[ToGoodC[JErrLAMBDA],"QuadDynamicModel_JErrLAMBDA.mthout"];
MyStringWrite[ToGoodC[JErrMU],"QuadDynamicModel_JErrMU.mthout"];
MyStringWrite[ToGoodC[JErrMC],"QuadDynamicModel_JErrMC.mthout"];
MyStringWrite[ToGoodC[JErrCP],"QuadDynamicModel_JErrCP.mthout"];
MyStringWrite[ToGoodC[JErrIBD],"QuadDynamicModel_JErrIBD.mthout"];
MyStringWrite[ToGoodC[JErrIBOD],"QuadDynamicModel_JErrIBOD.mthout"];


(* ::Subsubsection:: *)
(*2nd order derivatives*)


MyStringWrite[ToGoodC[AOut],"QuadDynamicModel_AOut.mthout"];
MyStringWrite[ToGoodC[AlphaOut],"QuadDynamicModel_AlphaOut.mthout"];


(* ::Subsection:: *)
(*FixMathematicaOutput*)


(* ::Subsubsection:: *)
(*State*)


Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_Err.mthout err 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrNoises.mthout J 0 0"];

Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrQ.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrV.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrW.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrA.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrAlpha.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrQprev.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrVprev.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrWprev.mthout J 0 0"];


(* ::Subsubsection:: *)
(*Parameters*)


Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrWINDXY.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrWINDZ.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrLDRAG.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrQDRAG.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrRDRAG.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrLAMBDA.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrMU.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrMC.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrCP.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrIBD.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py QuadDynamicModel_JErrIBOD.mthout J 0 0"];


(* ::Subsection:: *)
(*Final Steps*)


Run["mv *.cppready ../../../roamfree/ROAMfunctions/generated"];
Run["rm *.cppready"];
Run["rm *.mthout"];
