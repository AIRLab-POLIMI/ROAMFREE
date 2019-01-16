(* ::Package:: *)

GetDS[f_,params_]:=( 
	ASSUME=Join[Table[params[[i]]\[Element]Reals, {i, 1,Length[params]}],{t>=0,t\[Element]Reals}];
	vel=ParallelArray[Simplify[Refine[D[f[[#]],t],Assumptions:>ASSUME],ASSUME, TimeConstraint->{1,5}]&,3];
	Print["Vel completata"];
	acc=ParallelArray[Simplify[Refine[D[vel[[#]],t],Assumptions:>ASSUME],ASSUME, TimeConstraint->{1,5}]&,3];
	at=Projection[acc,vel];
	at=ParallelArray[Simplify[Refine[at[[#]],Assumptions:>ASSUME],ASSUME, TimeConstraint->{1,5}]&,3];
	Print["at completata"];
	an=(acc-at);
	an=ParallelArray[Simplify[Refine[an[[#]],Assumptions:>ASSUME],ASSUME, TimeConstraint->{1,5}]&,3];
	Print["an completata"];
	nan=Simplify[Refine[Norm[an],Assumptions:>ASSUME]//.Abs[p_]^2->p^2,ASSUME, TimeConstraint->{1,5}];
	nat=Simplify[Refine[Norm[at],Assumptions:>ASSUME]//.Abs[p_]^2->p^2,ASSUME, TimeConstraint->{1,5}];
	nv=Simplify[Refine[Norm[vel],Assumptions:>ASSUME]//.Abs[p_]^2->p^2,ASSUME, TimeConstraint->{1,5}];
	uv=vel/nv;
	uv=ParallelArray[Simplify[Refine[uv[[#]],Assumptions:>ASSUME],ASSUME, TimeConstraint->{1,5}]&,3];
	uan=an/nan;
	uan=ParallelArray[Simplify[Refine[uan[[#]],Assumptions:>ASSUME],ASSUME, TimeConstraint->{1,5}]&,3];
	e1=uv;
	e2=uan;
	e3=e1\[Cross]e2;
	e3=ParallelArray[Simplify[Refine[e3[[#]],Assumptions:>ASSUME],ASSUME, TimeConstraint->{1,5}]&,3];
	R=Transpose[Join[{e1},{e2},{e3}]];	
	(*
	Stefanini
	wtmp=D[R,t].Transpose[R];
	w={-wtmp[[2,3]],-wtmp[[1,3]],-wtmp[[1,2]]};*)
	(* Cucci *)
	wtmp=Transpose[R].D[R,t];
	w={-wtmp[[2,3]],wtmp[[1,3]],-wtmp[[1,2]]};
	w=ParallelArray[Simplify[Refine[w[[#]],Assumptions:>ASSUME],ASSUME, TimeConstraint->{1,5}]&,3];
	Print["w completata"];
	alpha=ParallelArray[Simplify[Refine[D[w[[#]],t],Assumptions:>ASSUME],ASSUME, TimeConstraint->{1,5}]&,3];
	Print["alpha completata"];
	wa=D[w,t];
	wa=ParallelArray[Simplify[Refine[wa[[#]],Assumptions:>ASSUME],ASSUME, TimeConstraint->{1,5}]&,3];
	Print["wa completata"];
	DSf={f,R,vel,w,acc,wa,alpha};
	Return[DSf];
);

RotToQuat[R_]:=(
	q={q1,q2,q3,q4};
	m1={q1,(R[[3,2]]-R[[2,3]])/(4*q1),(R[[1,3]]-R[[3,1]])/(4*q1),(R[[2,1]]-R[[1,2]])/(4*q1)};
	m2={ (R[[3,2]]-R[[2,3]])/(4*q2),q2,(R[[2,1]]+R[[1,2]])/(4*q2), (R[[1,3]]+R[[3,1]])/(4*q2)};
	m3={ (R[[1,3]]-R[[3,1]])/(4*q3), (R[[2,1]]+R[[1,2]])/(4*q3),q3, (R[[3,2]]+R[[2,3]])/(4*q3)};
	m4={ (R[[2,1]]-R[[1,2]])/(4*q4), (R[[1,3]]+R[[3,1]])/(4*q4),(R[[3,2]]+R[[2,3]])/(4*q4),q4};
	m={m1,m2,m3,m4};
	(* with Re 
	s1=Re[(1/2)*(Sqrt[1+R[[1,1]]+R[[2,2]]+R[[3,3]]])];
	s2=Re[(1/2)*(Sqrt[1+R[[1,1]]-R[[2,2]]-R[[3,3]]])];
	s3=Re[(1/2)*(Sqrt[1-R[[1,1]]+R[[2,2]]-R[[3,3]]])];
	s4=Re[(1/2)*(Sqrt[1-R[[1,1]]-R[[2,2]]+R[[3,3]]])];
	*)
	s1=(1/2)*(Sqrt[1+R[[1,1]]+R[[2,2]]+R[[3,3]]]);
	s2=(1/2)*(Sqrt[1+R[[1,1]]-R[[2,2]]-R[[3,3]]]);
	s3=(1/2)*(Sqrt[1-R[[1,1]]+R[[2,2]]-R[[3,3]]]);
	s4=(1/2)*(Sqrt[1-R[[1,1]]-R[[2,2]]+R[[3,3]]]);
	s={s1,s2,s3,s4};
	(* I don't think this needed?
	max=Flatten[Position[s,Max[s]]][[1]];
	*)
	max=1;
	Return[m[[max]]/.q[[max]]->s[[max]]];
);
