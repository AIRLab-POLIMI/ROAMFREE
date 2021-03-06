(* ::Package:: *)

QuatToRot[q_]:={{q[[1]]^2+q[[2]]^2-q[[3]]^2-q[[4]]^2,2*(q[[2]]*q[[3]]-q[[1]]*q[[4]]),2*(q[[2]]*q[[4]]+q[[1]]*q[[3]])},{2*(q[[2]]*q[[3]]+q[[1]]*q[[4]]),q[[1]]^2-q[[2]]^2+q[[3]]^2-q[[4]]^2,2*(q[[3]]*q[[4]]-q[[1]]*q[[2]])},{2*(q[[2]]*q[[4]]-q[[1]]*q[[3]]),2*(q[[3]]*q[[4]]+q[[1]]*q[[2]]),q[[1]]^2-q[[2]]^2-q[[3]]^2+q[[4]]^2}};


QuatProd[q1_,q2_]:={q1[[1]]*q2[[1]]-q1[[2]]*q2[[2]]-q1[[3]]*q2[[3]]-q1[[4]]*q2[[4]],q1[[1]]*q2[[2]]+q1[[2]]*q2[[1]]+q1[[3]]*q2[[4]]-q1[[4]]*q2[[3]],q1[[1]]*q2[[3]]-q1[[2]]*q2[[4]]+q1[[3]]*q2[[1]]+q1[[4]]*q2[[2]],q1[[1]]*q2[[4]]+q1[[2]]*q2[[3]]-q1[[3]]*q2[[2]]+q1[[4]]*q2[[1]]};


QuadProdMatrix[q_]:={{q[[1]],-q[[2]],-q[[3]],-q[[4]]},{q[[2]],q[[1]],-q[[4]],q[[3]]},{q[[3]],q[[4]],q[[1]],-q[[2]]},{q[[4]],-q[[3]],q[[2]],q[[1]]}};


QuatInv[q_]:={q[[1]],-q[[2]],-q[[3]],-q[[4]]};


QuatNorm[q_]:=q/(Sqrt[q[[1]]^2+q[[2]]^2+q[[3]]^2+q[[4]]^2]);


QuatInc[q_,dq_]:=QuatProd[q,{Sqrt[1-dq[[1]]^2-dq[[2]]^2-dq[[3]]^2],dq[[1]],dq[[2]],dq[[3]]}];


Qmat[q_]:={{q[[1]],-q[[2]],-q[[3]],-q[[4]]},
		{q[[2]],q[[1]],-q[[4]],q[[3]]},
		{q[[3]],q[[4]],q[[1]],-q[[2]]},
		{q[[4]],-q[[3]],q[[2]],q[[1]]}};
QPlusmat[q_]:={{q[[1]],-q[[2]],-q[[3]],-q[[4]]},
			{q[[2]],q[[1]],q[[4]],-q[[3]]},
			{q[[3]],-q[[4]],q[[1]],q[[2]]},
			{q[[4]],q[[3]],-q[[2]],q[[1]]}};


QuatBoxMinusRight[Q1_,Q2_]:= LinearSolve[Qmat[Q2],2(Q1-Q2)][[{2,3,4}]];
QuatBoxMinusLeft[Q1_,Q2_]:= LinearSolve[QPlusmat[Q2],2(Q1-Q2)][[{2,3,4}]];


QuatBoxPlusRight[Q1_,w_]:=Q1+1/2 Qmat[Q1].Flatten[{0,w}];
QuatBoxPlusLeft[Q1_,w_]:=Q1+1/2 QPlusmat[Q1].Flatten[{0,w}];
