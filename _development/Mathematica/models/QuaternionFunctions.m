(* ::Package:: *)

(* ::Subtitle:: *)
(*Quaternion Functions*)


QuatToRot[q_]:={{q[[1]]^2+q[[2]]^2-q[[3]]^2-q[[4]]^2,2*(q[[2]]*q[[3]]-q[[1]]*q[[4]]),2*(q[[2]]*q[[4]]+q[[1]]*q[[3]])},{2*(q[[2]]*q[[3]]+q[[1]]*q[[4]]),q[[1]]^2-q[[2]]^2+q[[3]]^2-q[[4]]^2,2*(q[[3]]*q[[4]]-q[[1]]*q[[2]])},{2*(q[[2]]*q[[4]]-q[[1]]*q[[3]]),2*(q[[3]]*q[[4]]+q[[1]]*q[[2]]),q[[1]]^2-q[[2]]^2-q[[3]]^2+q[[4]]^2}}


QuatProd[q1_,q2_]:={q1[[1]]*q2[[1]]-q1[[2]]*q2[[2]]-q1[[3]]*q2[[3]]-q1[[4]]*q2[[4]],q1[[1]]*q2[[2]]+q1[[2]]*q2[[1]]+q1[[3]]*q2[[4]]-q1[[4]]*q2[[3]],q1[[1]]*q2[[3]]-q1[[2]]*q2[[4]]+q1[[3]]*q2[[1]]+q1[[4]]*q2[[2]],q1[[1]]*q2[[4]]+q1[[2]]*q2[[3]]-q1[[3]]*q2[[2]]+q1[[4]]*q2[[1]]}


QuadProdMatrix[q_]:={{q[[1]],-q[[2]],-q[[3]],-q[[4]]},{q[[2]],q[[1]],-q[[4]],q[[3]]},{q[[3]],q[[4]],q[[1]],-q[[2]]},{q[[4]],-q[[3]],q[[2]],q[[1]]}};


QuatInv[q_]:={q[[1]],-q[[2]],-q[[3]],-q[[4]]}


VectProdMatrix[v_]:={{0,-v[[3]],v[[2]]},{v[[3]],0,-v[[1]]},{-v[[2]],v[[1]],0}}


QuatNorm[q_]:=q/(Sqrt[q[[1]]^2+q[[2]]^2+q[[3]]^2+q[[4]]^2])
QuatInc[q_,dq_]:=QuatProd[q,{Sqrt[1-dq[[1]]^2-dq[[2]]^2-dq[[3]]^2],dq[[1]],dq[[2]],dq[[3]]}]


PertStateSubs[X_,dX_]:=Flatten[{Table[X[[i]]->X[[i]]+dX[[i]],{i,1,3}],Table[X[[i+3]]->(QuatInc[X[[{4,5,6,7}]],dX[[{4,5,6}]]])[[i]],{i,1,4}]}]


L2[x_]:=Sqrt[Sum[x[[j]]^2,{j,1,Length[x]}]];
VecNorm[x_]:=Table[x[[i]]/L2[x],{i,1,Length[x]}];


SkewMatrix[v_]:={{0,-v[[1]],-v[[2]],-v[[3]]},{v[[1]],0,v[[3]],-v[[2]]},{v[[2]],-v[[3]],0,v[[1]]},{v[[3]],v[[2]],-v[[1]],0}}/2;


ConstantAngularSpeedTMatrix[w_,dt_]:=Cos[L2[w]dt/2] IdentityMatrix[4]+2Sin[L2[w]dt/2]/L2[w] SkewMatrix[w];


SubVec[exp_,v1_,v2_]:=exp/.Table[v1[[i]]->v2[[i]],{i,1,Dimensions[v1,1][[1]]}];
VecToZero[exp_,v_]:=exp/.Table[v[[i]]->0,{i,1,Dimensions[v,1][[1]]}];


ToGoodC[exp_]:=Module[{oexp}, oexp=Experimental`OptimizeExpression[exp];
If[Dimensions [StringPosition[ToString[InputForm[oexp]],"Compile"],1][[1]]>0,Block[{ locals, code},{locals,code}=ReleaseHold[(Hold@@oexp)/.Verbatim[Block][vars_,seq_]:>{vars,Hold[seq]}];ToString[CForm[code]]]
, ToString[CForm[exp]]]]
MyStringWrite[str_,file_]:=Module[{stream},stream=OpenWrite[file];WriteString[stream,str];Close[stream];]
