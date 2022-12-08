function [qd, qd_dot, qd_dotdot] = get_desired_trajectory(q0,q0_dot,q0_dotdot,qf,qf_dot,qf_dotdot,tf,step,n)

    % - - - - - - - - - - - - - - - - - - -
    % input:
    % q0: Initial Position
    % q0_dot: Initial Velocity
    % q0_dotdot: Initial Acceleration
    % qf: Final Position
    % qf_dot: Final Velocity
    % qf_dotdot: Final Acceleration
    % tf: Final Time
    % step: Step
    % n: Robot Links
    % - - - - - - - - - - - - - - - - - - -
    % output:
    % qd: [qd(1),qd(2),...,qd(n)]
    % qd_dot: [qd_dot(1),qd_dot(2),...,qd_dot(n)]
    % qd_dotdot: [qd_dotdot(1),qd_dotdot(2),...,qd_dotdot(n)]
    % - - - - - - - - - - - - - - - - - - -

    qd=zeros(n,tf*(1/step));
    qd_dot=zeros(n,tf*(1/step));
    qd_dotdot=zeros(n,tf*(1/step));

    qdt=sym('qdt',[n 1],'real');
    qdt_dot=sym('qdt_dot',[n 1],'real');
    qdt_dotdot=sym('qdt_dotdot',[n 1],'real');

    for i=1:n

        a=sym('a',[1 6],'real');
        s=sym('s','real');
        fivep=a(1)*s^5+a(2)*s^4+a(3)*s^3+a(4)*s^2+a(5)*s+a(6);
        eq1=subs(fivep,s,1);
        eq2=subs(fivep,s,tf*(1/step));
        eq3=subs(diff(fivep,s,1),s,1);
        eq4=subs(diff(fivep,s,1),s,tf*(1/step));
        eq5=subs(diff(fivep,s,2),s,1);
        eq6=subs(diff(fivep,s,2),s,tf*(1/step));
        result=struct2array(solve([eq1==q0(i) eq2==qf(i) eq3==q0_dot(i) eq4==qf_dot(i) eq5==q0_dotdot(i) eq6==qf_dotdot(i)],a));

        P=subs(fivep,a,result);

        qdt(i)=P;
        qdt_dot(i)=(1/step)*diff(P,s,1);
        qdt_dotdot(i)=(1/step)^2*diff(P,s,2);

    end

    for i=1:tf*(1/step)
        
        t=i;
        qd(:,i)=double(subs(qdt,'s',t));
        qd_dot(:,i)=double(subs(qdt_dot,'s',t));
        qd_dotdot(:,i)=double(subs(qdt_dotdot,'s',t));

    end

end