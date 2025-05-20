X_Start = [1,0,0,0;
           0,1,0,0;
           0,0,1,0;
           0,0,0,1;
];

X_End = [0,0,1,1;
         1,0,0,2;
         0,1,0,3;
         0,0,0,1;
];

 TF = 10;

 method = 5;

 N = 10;

 Result_Question6 = ScrewTrajectory(X_Start,X_End,TF,N,method);

 Result_Question7 = CartesianTrajectory(X_Start,X_End,TF,N,method);
