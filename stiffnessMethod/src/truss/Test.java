/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package truss;

/**
 *
 * @author Kamran
 */

public class Test {

    public static void main(String args[]) {
    /*   
        //Define Joints
        Joint j1 = new Joint(0, 0);
        Joint j2 = new Joint(10, 0);
        Joint j3 = new Joint(10, 10);
        Joint j4 = new Joint(0, 10);
        
        //Apply support condition
        j2.setxRestrained(true);
        //j2.setyRestrained(true);
        j3.setxRestrained(true);
        j3.setyRestrained(true);
        
        
        //construct members
        Member m1 = new Member(j1, j2);
        Member m2 = new Member(j1, j3);
        Member m3 = new Member(j1, j4);
        Member m4 = new Member(j4, j3);
        Member m5 = new Member(j4, j2);
        Member m6 = new Member(j2, j3);
        
        //construct model
        Model model = new Model();
        model.getMemberAl().add(m1);
        model.getMemberAl().add(m2);
        model.getMemberAl().add(m3);
        model.getMemberAl().add(m4);
        model.getMemberAl().add(m5);
        model.getMemberAl().add(m6);
        //Apply loads on joints
        model.jXLoads(j1, 0);
        model.jYLoads(j1, 0);
        model.jXLoads(j4, 2);
        model.jYLoads(j4, -4);
        model.jYLoads(j2, 0);
        
   */
       // Create joints
        Joint j1 = new Joint(0, 0);
        Joint j2=new Joint(3,0);
        Joint j3 = new Joint(3, 4);
        j2.setxRestrained(true);
        j2.setyRestrained(true);
        j3.setxRestrained(true);
        j3.setyRestrained(true);
        
        //create  members
        Member m1=new Member(j1,j2);
        Member m2=new Member(j1,j3);
        Model model=new Model();
        model.getMemberAl().add(m1);
        model.getMemberAl().add(m2);
        model.jYLoads(j1, -2);
        model.jXLoads(j1, 0);
        
        int i = 1;
        for (Member m : model.getMemberAl()) {
            System.out.println("m" + i++ + "=" + m.toString());
        }

        model.populateJointAl();
        int j=1;
        for (int k=0;k<model.getJointAl().size();k++){
            System.out.println("j"+j++ +"="+model.getJointAl().get(k).toString());
        }
        
        Compute c=new Compute(model);
                
        
    }
}
