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


(* ::Section:: *)
(*Noises and Inputs*)


Z=Table[Abs[z[i]],{i,1,3}];
NOISES=Table[eta[i],{i,1,3}];
ZNoise = Z + Table[NOISES[[i]],{i,1,3}];


(* ::PageBreak:: *)
(**)


(* ::Section:: *)
(*Equations*)


(* ::Subsection:: *)
(*Errors*)


Ahat = ZNoise A;
ERR = Flatten[{A-Ahat}];


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
JErrP =VecToZero[D[ERR,{POSE}]/.Reg,NOISES];
JErrQ=VecToZero[D[ERR,{Q}]/.Reg,NOISES];
JErrV=VecToZero[D[ERR,{V}]/.Reg,NOISES];
JErrW=VecToZero[D[ERR,{W}]/.Reg,NOISES];
JErrA=VecToZero[D[ERR,{A}]/.Reg,NOISES];
JErrAlpha=VecToZero[D[ERR,{Alpha}]/.Reg,NOISES];


(* ::Subsubsection:: *)
(*2nd order derivatives*)


AOut = VecToZero[Ahat,NOISES];


(* ::Subsection:: *)
(*Mthout files*)


Run["rm *.cppready"];


(* ::Subsubsection:: *)
(*State*)


MyStringWrite[ToGoodC[Err],"PoseDerivative_Err.mthout"];
MyStringWrite[ToGoodC[JErrNoises],"PoseDerivative_JErrNoises.mthout"];
MyStringWrite[ToGoodC[JErrP],"PoseDerivative_JErrP.mthout"];
MyStringWrite[ToGoodC[JErrQ],"PoseDerivative_JErrQ.mthout"];
MyStringWrite[ToGoodC[JErrV],"PoseDerivative_JErrV.mthout"];
MyStringWrite[ToGoodC[JErrW],"PoseDerivative_JErrW.mthout"];
MyStringWrite[ToGoodC[JErrA],"PoseDerivative_JErrA.mthout"];
MyStringWrite[ToGoodC[JErrAlpha],"PoseDerivative_JErrAlpha.mthout"];


(* ::Subsubsection:: *)
(*2nd order derivatives*)


MyStringWrite[ToGoodC[AOut],"PoseDerivative_AOut.mthout"];


(* ::Subsection:: *)
(*FixMathematicaOutput*)


(* ::Subsubsection:: *)
(*State*)


Run["python ../fixMathematicaOutput_v2.py PoseDerivative_Err.mthout err 0 0"];
Run["python ../fixMathematicaOutput_v2.py PoseDerivative_JErrNoises.mthout J 0 0"];

Run["python ../fixMathematicaOutput_v2.py PoseDerivative_JErrP.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PoseDerivative_JErrQ.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PoseDerivative_JErrV.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PoseDerivative_JErrW.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PoseDerivative_JErrA.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PoseDerivative_JErrAlpha.mthout J 0 0"];


(* ::Subsection:: *)
(*Final Steps*)


Run["mv *.cppready ../../../roamfree/ROAMfunctions/generated"];
Run["rm *.cppready"];
Run["rm *.mthout"];
