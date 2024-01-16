#!/usr/bin/env wolframscript
(* ::Package:: *)

Clear["Global`*"]


Import[NotebookDirectory[]<>"SimplifyFunctions.m"]
Import[NotebookDirectory[]<>"QuaternionAlgebra.m"]


(* ::Subtitle:: *)
(*General Functions*)


L2[x_]:=Sqrt[Sum[x[[j]]^2,{j,1,Length[x]}]];
VecNorm[x_]:=Table[x[[i]]/L2[x],{i,1,Length[x]}];


SkewMatrix4[v_]:={{0,-v[[1]],-v[[2]],-v[[3]]},{v[[1]],0,v[[3]],-v[[2]]},{v[[2]],-v[[3]],0,v[[1]]},{v[[3]],v[[2]],-v[[1]],0}}/2;
SkewMatrix[v_]:={{0,-v[[3]],v[[2]]},{v[[3]],0,-v[[1]]},{-v[[2]],v[[1]],0}}


SubVec[exp_,v1_,v2_]:=exp/.Table[v1[[i]]->v2[[i]],{i,1,Dimensions[v1,1][[1]]}];
VecToZero[exp_,v_]:=exp/.Table[v[[i]]->0,{i,1,Dimensions[v,1][[1]]}];


ToGoodC[exp_]:=Module[{oexp}, oexp=Experimental`OptimizeExpression[exp];
If[Dimensions [StringPosition[ToString[InputForm[oexp]],"Compile"],1][[1]]>0,Block[{ locals, code},{locals,code}=ReleaseHold[(Hold@@oexp)/.Verbatim[Block][vars_,seq_]:>{vars,Hold[seq]}];ToString[CForm[code]]]
, ToString[CForm[exp]]]]
MyStringWrite[str_,file_]:=Module[{stream},stream=OpenWrite[file];WriteString[stream,str];Close[stream];]


(* ::Subtitle:: *)
(*Quaternion Functions*)


PertStateSubs[X_,dX_]:=Flatten[{Table[X[[i]]->X[[i]]+dX[[i]],{i,1,3}],Table[X[[i+3]]->(QuatInc[X[[{4,5,6,7}]],dX[[{4,5,6}]]])[[i]],{i,1,4}]}]


ConstantAngularSpeeDtMatrix[w_,Dt_]:=Cos[L2[w]Dt/2] IdentityMatrix[4]+2Sin[L2[w]Dt/2]/L2[w] SkewMatrix[w];


(* ::Subtitle:: *)
(*Beginning of stuff*)


(* ::Text:: *)
(*State*)


X=Table[x[i],{i,1,33}];


QWS=X[[4;;7]];
Disp = X[[{20,21,22}]];


QWSprev = X[[30;;33]];


Z2 = Table[z[i],{i,1,3}];
Z1 = Table[z[i],{i,4,5}];


NOISES=Table[eta[i],{i,1,2}];


(* ::Text:: *)
(*Parameters*)


QL2B = {lqOS[1], lqOS[2], lqOS[3], lqOS[4]};
tL2B = {lSO[1],lSO[2], lSO[3]};
QC2B = {cqOS[1], cqOS[2], cqOS[3], cqOS[4]};
tC2B = {cSO[1], cSO[2], cSO[3]};


(* intrinsic camera calibration matrix *)
CMv={cm[1],cm[2],cm[3]};
CM={{cm[1],0,cm[2]},{0, cm[1], cm[3]},{0,0,1}}; 


(* radial, tangential and skew distortion coefficients *)
RD ={rd[1],rd[2],rd[3], rd[4]};
TD ={td[1],td[2]};
SKEW={sk[1], sk[2]};


(* ::Text:: *)
(*Equations*)


VecFromLidarInBody=QuatToRot[QL2B] . Z2 + tL2B; (* lidar vector to the body frame *)
VecFromBody2PrevBody=QuatToRot[QuatProd[QuatInv[QWSprev],QWS]] . VecFromLidarInBody+Disp; (* lidar vector to the body frame from which camera tp is taken *)
VecFromPrevBody2PrevCam = QuatToRot[QuatInv[QC2B]] . (VecFromBody2PrevBody - tC2B); (* lidar vector to the camera frame *)


LwC=VecFromPrevBody2PrevCam;


(* with distortion model according to https://www.agisoft.com/pdf/metashape-pro_1_7_en.pdf*)
xu=LwC[[1]]/LwC[[3]];
yu=LwC[[2]]/LwC[[3]];
rq=xu^2+yu^2;
radial=RD[[1]]*rq+RD[[2]]*rq^2+RD[[3]]*rq^3+RD[[4]]*rq^4;


xd=xu(1+radial)+(TD[[1]]*(rq+2*xu^2)+2*TD[[2]]*xu*yu);
yd=yu(1+radial)+(TD[[2]]*(rq+2*yu^2)+2*TD[[1]]*xu*yu);


ZhatNormDistFull={cm[2]+(cm[1]+SKEW[[1]])*xd+SKEW[[2]]*yd ,cm[3]+cm[1]*yd };


ZhatNormDist=ZhatNormDistFull;


ERRlong = Z1-ZhatNormDist+NOISES;
ERRlong//MatrixForm


ERR = SimplifyQ[ERRlong,{QWS, QWSprev},AreReals[{Disp, Z1, Z2,NOISES}]];
ERR//MatrixForm


(* ::Text:: *)
(*Differentiation variables*)


dX={dqx,dqy,dqz};
dXQScaling= 0.01;


(* ::Text:: *)
(*Output*)


SetDirectory[NotebookDirectory[]];


Err = VecToZero[ERR,NOISES];
JErrNoises=VecToZero[D[ERR,{NOISES}],NOISES];


JErrDisp=D[Err,{Disp}];
JErrQ=D[Err,{QWS}];
JErrQprev=D[Err,{QWSprev}];


QL2Bpert = QuatInc[QL2B,dXQScaling*dX];
JErrQL2B=VecToZero[D[SubVec[Err,QL2B,QL2Bpert],{dX}],dX];
JErrtL2B=D[Err,{tL2B}];
QC2Bpert = QuatInc[QC2B,dXQScaling*dX];
JErrQC2B=VecToZero[D[SubVec[Err,QC2B,QC2Bpert],{dX}],dX];
JErrtC2B=D[Err,{tC2B}];


JErrCM=D[Err,{CMv}];
JErrRD=D[Err,{RD}];
JErrTD=D[Err,{TD}];
JErrSKEW=D[Err,{SKEW}];


Run["rm *.cppready"];


MyStringWrite[ToGoodC[Err],"LiDAR2ImageTieFeature_Err.mthout"];
MyStringWrite[ToGoodC[JErrNoises],"LiDAR2ImageTieFeature_JErrNoises.mthout"];
MyStringWrite[ToGoodC[JErrDisp],"LiDAR2ImageTieFeature_JErrDisp.mthout"];
MyStringWrite[ToGoodC[JErrQ],"LiDAR2ImageTieFeature_JErrQ.mthout"];
MyStringWrite[ToGoodC[JErrQprev],"LiDAR2ImageTieFeature_JErrQprev.mthout"];
MyStringWrite[ToGoodC[JErrQL2B],"LiDAR2ImageTieFeature_JErrQL2B.mthout"];
MyStringWrite[ToGoodC[JErrtL2B],"LiDAR2ImageTieFeature_JErrtL2B.mthout"];
MyStringWrite[ToGoodC[JErrQC2B],"LiDAR2ImageTieFeature_JErrQC2B.mthout"];
MyStringWrite[ToGoodC[JErrtC2B],"LiDAR2ImageTieFeature_JErrtC2B.mthout"];
MyStringWrite[ToGoodC[JErrCM],"LiDAR2ImageTieFeature_JErrCM.mthout"];
MyStringWrite[ToGoodC[JErrRD],"LiDAR2ImageTieFeature_JErrRD.mthout"];
MyStringWrite[ToGoodC[JErrTD],"LiDAR2ImageTieFeature_JErrTD.mthout"];
MyStringWrite[ToGoodC[JErrSKEW],"LiDAR2ImageTieFeature_JErrSKEW.mthout"];


Run["python3 ../fixMathematicaOutput_v2.py LiDAR2ImageTieFeature_Err.mthout err 0 0"];
Run["python3 ../fixMathematicaOutput_v2.py LiDAR2ImageTieFeature_JErrNoises.mthout J 0 0"];
Run["python3 ../fixMathematicaOutput_v2.py LiDAR2ImageTieFeature_JErrDisp.mthout J 0 0"];
Run["python3 ../fixMathematicaOutput_v2.py LiDAR2ImageTieFeature_JErrQ.mthout J 0 0"];
Run["python3 ../fixMathematicaOutput_v2.py LiDAR2ImageTieFeature_JErrQprev.mthout J 0 0"];
Run["python3 ../fixMathematicaOutput_v2.py LiDAR2ImageTieFeature_JErrQL2B.mthout J 0 0"];
Run["python3 ../fixMathematicaOutput_v2.py LiDAR2ImageTieFeature_JErrtL2B.mthout J 0 0"];
Run["python3 ../fixMathematicaOutput_v2.py LiDAR2ImageTieFeature_JErrQC2B.mthout J 0 0"];
Run["python3 ../fixMathematicaOutput_v2.py LiDAR2ImageTieFeature_JErrtC2B.mthout J 0 0"];
Run["python3 ../fixMathematicaOutput_v2.py LiDAR2ImageTieFeature_JErrCM.mthout J 0 0"];
Run["python3 ../fixMathematicaOutput_v2.py LiDAR2ImageTieFeature_JErrRD.mthout J 0 0"];
Run["python3 ../fixMathematicaOutput_v2.py LiDAR2ImageTieFeature_JErrTD.mthout J 0 0"];
Run["python3 ../fixMathematicaOutput_v2.py LiDAR2ImageTieFeature_JErrSKEW.mthout J 0 0"];


Run["mv *.cppready ../../../roamfree/ROAMfunctions/generated"];


Run["rm *.cppready"];
Run["rm *.mthout"];
