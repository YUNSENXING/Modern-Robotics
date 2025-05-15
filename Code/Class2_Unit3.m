% Unit3 Chapter 6
Slist = [0, 0, 0;
         0, 0, 0;
         1, 1, 1;
         0, 0, 0;
         0, -1, -2;
         0, 0, 0
];

M = [1, 0, 0, 3;
     0, 1, 0, 0;
     0, 0, 1, 0;
     0, 0, 0, 1
];

T = [-0.585, -0.811, 0, 0.076;
     0.811, -0.585, 0. 2.608;
     0, 0, 1, 0;
     0, 0, 0, 1;
];

Theta = [pi / 4; pi / 4; pi / 4];


eomg = 0.001;

ev = 0.0001;

[Answer,success] = IKinSpace(Slist,M,T,Theta,eomg,ev);


disp(Answer);

disp(success);

