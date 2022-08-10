#!/usr/bin/env wolframscript
(* ::Package:: *)

Clear["Global`*"]



parentPath = $InputFileName /. "" :> NotebookFileName[]
parentDir = DirectoryName @ parentPath
Import[parentDir<>"SimplifyFunctions.m"]
Import[parentDir<>"QuaternionAlgebra.m"]



(* ::Title:: *)
(* Useful Functions *)


VectProdMatrix[v_]:={{0,-v[[3]],v[[2]]},{v[[3]],0,-v[[1]]},{-v[[2]],v[[1]],0}}


PertStateSubs[X_,dX_]:=Flatten[{Table[X[[i]]->X[[i]]+dX[[i]],{i,1,3}],Table[X[[i+3]]->(QuatInc[X[[{4,5,6,7}]],dX[[{4,5,6}]]])[[i]],{i,1,4}]}]


L2[x_]:=Sqrt[Sum[x[[j]]^2,{j,1,Length[x]}]];
VecNorm[x_]:=Table[x[[i]]/L2[x],{i,1,Length[x]}];


SkewMatrix[v_]:={{0,-v[[1]],-v[[2]],-v[[3]]},{v[[1]],0,v[[3]],-v[[2]]},{v[[2]],-v[[3]],0,v[[1]]},{v[[3]],v[[2]],-v[[1]],0}}/2;


ConstantAngularSpeeDtMatrix[w_,Dt_]:=Cos[L2[w]Dt/2] IdentityMatrix[4]+2Sin[L2[w]Dt/2]/L2[w] SkewMatrix[w];


SubVec[exp_,v1_,v2_]:=exp/.Table[v1[[i]]->v2[[i]],{i,1,Dimensions[v1,1][[1]]}];
VecToZero[exp_,v_]:=exp/.Table[v[[i]]->0,{i,1,Dimensions[v,1][[1]]}];


ToGoodC[exp_]:=Module[{oexp}, oexp=Experimental`OptimizeExpression[exp];
If[Dimensions [StringPosition[ToString[InputForm[oexp]],"Compile"],1][[1]]>0,Block[{ locals, code},{locals,code}=ReleaseHold[(Hold@@oexp)/.Verbatim[Block][vars_,seq_]:>{vars,Hold[seq]}];ToString[CForm[code]]]
, ToString[CForm[exp]]]]
MyStringWrite[str_,file_]:=Module[{stream},stream=OpenWrite[file];WriteString[stream,str];Close[stream];]


(* ::Title:: *)
(* Beginning of stuff *)


(* ::Subsubtitle:: *)
(* State *)


X=Table[x[i],{i,1,39}];
SW=X[[1;;3]];
QWS=X[[4;;7]];
VS=X[[8;;10]];
WS=X[[11;;13]];
AS=X[[14;;16]];
ALPHAS=X[[17;;19]];
QWSprev = X[[30;;33]];
VSprev = X[[34;;36]];
WSprev = X[[37;;39]];


(* ::Subsubtitle:: *)
(* Parameters *)


CFT1=fThrust[1]; (* thrust force coefficients*);
CFT2=fThrust[2];
CFT3=fThrust[3];

CFX1=fDrag[1]; (* drag force coefficients *);
CFXalpha1 = fDrag[2];
CFXalpha2 = fDrag[3];
CFXbeta2 = fDrag[4];

CFY1=fLat[1];  (* lateral force coefficient *);

CFZ1=fLift[1]; (* lift force coefficients *);
CFZalpha=fLift[2];

CMXa=mRoll[1];(* roll moment coefficients *);
CMXbeta=mRoll[2];
CMX\[Omega]tildex=mRoll[3];
CMX\[Omega]tildez=mRoll[4];

CMY1=mPitch[1];(* pitch moment coefficients *);
CMYe=mPitch[2];
CMYalpha=mPitch[3];
CMY\[Omega]tildey=mPitch[4];

CMZr=mYaw[1]; (* yaw moment coefficients *);
CMZbeta=mYaw[2];
CMZ\[Omega]tildez=mYaw[3];

cbar = cBar[1]; (* mean aereodynamic chord *);

PD=cp[1];  (* propeller diameter *);
S=cp[2]; (* wing surface *);
b=cp[3]; (* wind span *);
m = cp[4]; (* mass *);

Ixx=ibd[1];
Iyy=ibd[2];
Izz =ibd[3];

Ixz=ibod[1];
Ixy=ibod[2];
Iyz=ibod[3];



IB={{Ixx,Ixy,Ixz},{Ixy,Iyy,Iyz},{Ixz,Iyz,Izz}}; (* Inertia matrix *);
AIRDENSITY = Table[airDensity[i],{i,1,3}];


FTHRUST = Table[fThrust[i],{i,1,3}];
FDRAG = Table[fDrag[i],{i,1,4}];
FLAT = Table[fLat[i],{i,1,1}];
FLIFT= Table[fLift[i],{i,1,2}];

MROLL = Table[mRoll[i],{i,1,4}];
MPITCH = Table[mPitch[i],{i,1,4}];
MYAW = Table[mYaw[i],{i,1,3}];

CBAR = Table[cBar[i],{i,1,1}];

CP =  Table[cp[i],{i,1,4}];

IBD =  Table[ibd[i],{i,1,3}];
IBOD =  Table[ibod[i],{i,1,3}];



(* wind speed in world frame *)
(* Wind planar in world coordinates *)
WINDXY = {windxy[1],windxy[2]};
(* Wind vertical in world coordinates *)
WINDZ = {windz[1]};
(* Wind in world coordinates *)
WIND={windxy[1],windxy[2],windz[1]}; 



NOISES=Table[eta[i],{i,1,6}];
NOISESCONTROLS = Table[NOISES[[i]],{i,{1,4,5,6}}];
EXTRANOISES ={0, NOISES[[2]], NOISES[[3]], 0, 0, 0};



n = z[1]+NOISESCONTROLS[[1]]; (* propeller speed *);
\[Delta]a=z[2]+NOISESCONTROLS[[2]];(* aileron deflection *);
\[Delta]e=z[3]+NOISESCONTROLS[[3]];(* elevator deflection *);
\[Delta]r=z[4]+NOISESCONTROLS[[4]];(* rudder deflection *);



(* ::Subsubtitle:: *)
(* Equations *)


(* ::Text:: *)
(* Air density model *)


(*T=T0(1+a (h0+SW[[3]])/T0);
\[Rho]=p0(1+a(h0+SW[[3]])/T0)^5.2561/(Ra T)//FullSimplify;*)
\[Rho] = 1.268;


(* ::Text:: *)
(* Airspeed *)


AIRS=VSprev-QuatToRot[QuatInv[QWSprev]] . WIND;
\[Alpha]=ArcTan[AIRS[[1]],AIRS[[3]]];
\[Beta]=ArcSin[AIRS[[2]]/L2[AIRS]]; 
(* dynamic pressure *)
qbar=\[Rho] L2[AIRS]^2/2;


(* Wind frame  *)
(*RWINDS=Transpose[RotationMatrix[\[Alpha],{0,1,0}]].RotationMatrix[\[Beta],{0,0,1}];*)
RWINDS = {{Cos[\[Alpha]],0,-Sin[\[Alpha]]},{0,1,0},{Sin[\[Alpha]],0,Cos[\[Alpha]]}} . {{Cos[\[Beta]],-Sin[\[Beta]],0},{Sin[\[Beta]],Cos[\[Beta]],0},{0,0,1}};


(* ::Text:: *)
(* Thrust force *)


(*this can cause division by zero*)
(*CFT[J_]:=CFT1+CFT2 J+ CFT3 J^2;
FT=\[Rho] n^2 PD^4 CFT[L2[AIRS]/(PD \[Pi] n)];*)
FT=\[Rho] PD^2 (CFT1 PD^2 n^2 + CFT2 PD n L2[AIRS]/\[Pi]+CFT3 L2[AIRS]^2/\[Pi]^2);


(* ::Text:: *)
(* Drag force *)


CFX=CFX1-CFXalpha1 \[Alpha]+CFXalpha2 \[Alpha]^2+CFXbeta2 \[Beta]^2;
FWX=qbar S CFX;


(* ::Text:: *)
(* Lateral force *)


CFY=CFY1 \[Beta];
FWY=qbar S CFY;


(* ::Text:: *)
(* Lift force *)


CFZ=-CFZ1+CFZalpha \[Alpha];
FWZ=qbar S CFZ;


(* ::Text:: *)
(* Roll moment *)


\[Omega]tildex= b WSprev[[1]]/(2 L2[AIRS]);
\[Omega]tildez=b WSprev[[3]]/(2 L2[AIRS]);
CMX=CMXa \[Delta]a -CMXbeta \[Beta]+CMX\[Omega]tildex \[Omega]tildex -CMX\[Omega]tildez \[Omega]tildez;
MBX=qbar S b CMX;


(* ::Text:: *)
(* Pitch moment *)


\[Omega]tildey=cbar WSprev[[2]]/(2 L2[AIRS]);
CMY=-CMY1-CMYe \[Delta]e +CMY\[Omega]tildey \[Omega]tildey+CMYalpha \[Alpha];
MBY=qbar S cbar CMY;


(* ::Text:: *)
(* Yaw moment *)


CMZ = -CMZr \[Delta]r + CMZ\[Omega]tildez \[Omega]tildez +CMZbeta \[Beta];
MBZ=qbar S b CMZ;


(* ::Text:: *)
(* Error equations *)


forceBfPrev = {FT,0,0}+RWINDS . {FWX,FWY,FWZ};
momentBfPrev = {MBX,MBY,MBZ};

Ahatlong = 1/m (forceBfPrev);
AlphaHatlong= Inverse[IB] . (momentBfPrev-WSprev\[Cross](IB . WSprev));

(* take into account the fact that these accelerations are computed using current state, but will be integrated to get the next state *)
Ahatlong = QuatToRot[QuatInv[QWS]] . QuatToRot[QWSprev] . Ahatlong
AlphaHatlong = QuatToRot[QuatInv[QWS]] . QuatToRot[QWSprev] . AlphaHatlong


Ahat=Ahatlong;
Alphahat=AlphaHatlong;


ERR=Flatten[{AS-Ahat,ALPHAS-Alphahat}]+EXTRANOISES;



(* ::Subsubtitle:: *)
(* Predictor *)


(* until we fix the predictor interface we just predict that we move forward at 1 m/s *)
SWhat=SW+QuatToRot[QWS] . ({1*dt, 0,0});
QWShat=QWS;



(* ::Subsubtitle:: *)
(* Output *)


SetDirectory[parentDir];



Err = VecToZero[ERR,NOISES];
JErrNoises=VecToZero[D[ERR,{NOISES}],NOISES];
JErrPose=VecToZero[D[ERR,{SW}],NOISES];
JErrQ=VecToZero[D[ERR,{QWS}],NOISES];
JErrV=VecToZero[D[ERR,{VS}],NOISES];
JErrW=VecToZero[D[ERR,{WS}],NOISES];
JErrA=VecToZero[D[ERR,{AS}],NOISES];
JErrAlpha=VecToZero[D[ERR,{ALPHAS}],NOISES];
JErrQprev=VecToZero[D[ERR,{QWSprev}],NOISES];
JErrVprev=VecToZero[D[ERR,{VSprev}],NOISES];
JErrWprev=VecToZero[D[ERR,{WSprev}],NOISES];



JErrAirDensity=VecToZero[D[ERR,{AIRDENSITY}],NOISES];

JErrFThrust=VecToZero[D[ERR,{FTHRUST}],NOISES];
JErrFDrag=VecToZero[D[ERR,{FDRAG}],NOISES];
JErrFLat=VecToZero[D[ERR,{FLAT}],NOISES];
JErrFLift=VecToZero[D[ERR,{FLIFT}],NOISES];

JErrMRoll=VecToZero[D[ERR,{MROLL}],NOISES];
JErrMPitch=VecToZero[D[ERR,{MPITCH}],NOISES];
JErrMYaw=VecToZero[D[ERR,{MYAW}],NOISES];

JErrCBar=VecToZero[D[ERR,{CBAR}],NOISES];

JErrCp=VecToZero[D[ERR,{CP}],NOISES];
JErrIbd=VecToZero[D[ERR,{IBD}],NOISES];
JErrIbod=VecToZero[D[ERR,{IBOD}],NOISES];

JErrWindXY=VecToZero[D[ERR,{WINDXY}],NOISES];
JErrWindZ=VecToZero[D[ERR,{WINDZ}],NOISES];




Run["rm *.cppready"];

MyStringWrite[ToGoodC[JErrNoises],"PlaneDynamicModel_JErrNoises.mthout"];
MyStringWrite[ToGoodC[Err],"PlaneDynamicModel_Err.mthout"];
MyStringWrite[ToGoodC[JErrNoises],"PlaneDynamicModel_JErrNoises.mthout"];
MyStringWrite[ToGoodC[JErrPose],"PlaneDynamicModel_JErrPose.mthout"];
MyStringWrite[ToGoodC[JErrQ],"PlaneDynamicModel_JErrQ.mthout"];
MyStringWrite[ToGoodC[JErrV],"PlaneDynamicModel_JErrV.mthout"];
MyStringWrite[ToGoodC[JErrW],"PlaneDynamicModel_JErrW.mthout"];
MyStringWrite[ToGoodC[JErrA],"PlaneDynamicModel_JErrA.mthout"];
MyStringWrite[ToGoodC[JErrAlpha],"PlaneDynamicModel_JErrAlpha.mthout"];
MyStringWrite[ToGoodC[JErrQprev],"PlaneDynamicModel_JErrQprev.mthout"];
MyStringWrite[ToGoodC[JErrVprev],"PlaneDynamicModel_JErrVprev.mthout"];
MyStringWrite[ToGoodC[JErrWprev],"PlaneDynamicModel_JErrWprev.mthout"];
MyStringWrite[ToGoodC[JErrAirDensity],"PlaneDynamicModel_JErrAirDensity.mthout"];
MyStringWrite[ToGoodC[JErrFThrust],"PlaneDynamicModel_JErrFThrust.mthout"];
MyStringWrite[ToGoodC[JErrFDrag],"PlaneDynamicModel_JErrFDrag.mthout"];
MyStringWrite[ToGoodC[JErrFLat],"PlaneDynamicModel_JErrFLat.mthout"];
MyStringWrite[ToGoodC[JErrFLift],"PlaneDynamicModel_JErrFLift.mthout"];
MyStringWrite[ToGoodC[JErrMRoll],"PlaneDynamicModel_JErrMRoll.mthout"];
MyStringWrite[ToGoodC[JErrMPitch],"PlaneDynamicModel_JErrMPitch.mthout"];
MyStringWrite[ToGoodC[JErrMYaw],"PlaneDynamicModel_JErrMYaw.mthout"];
MyStringWrite[ToGoodC[JErrCBar],"PlaneDynamicModel_JErrCBar.mthout"];
MyStringWrite[ToGoodC[JErrCp],"PlaneDynamicModel_JErrCp.mthout"];
MyStringWrite[ToGoodC[JErrIbd],"PlaneDynamicModel_JErrIbd.mthout"];
MyStringWrite[ToGoodC[JErrIbod],"PlaneDynamicModel_JErrIbod.mthout"];
MyStringWrite[ToGoodC[JErrWindXY],"PlaneDynamicModel_JErrWindXY.mthout"];
MyStringWrite[ToGoodC[JErrWindZ],"PlaneDynamicModel_JErrWindZ.mthout"];

MyStringWrite[ToGoodC[Flatten[{SWhat,QWShat}]],"PlaneDynamicModel_predictor.mthout"];


Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_Err.mthout err 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrNoises.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrPose.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrQ.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrV.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrW.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrA.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrAlpha.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrQprev.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrVprev.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrWprev.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrAirDensity.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrFThrust.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrFDrag.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrFLat.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrFLift.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrMRoll.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrMPitch.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrMYaw.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrCBar.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrCp.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrIbd.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrIbod.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrWindXY.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_JErrWindZ.mthout J 0 0"];

Run["python ../fixMathematicaOutput_v2.py PlaneDynamicModel_predictor.mthout xhat 0 0"];



Run["mv *.cppready ../../../roamfree/ROAMfunctions/generated"]
Run["rm *.mthout"];



