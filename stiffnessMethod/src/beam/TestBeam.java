/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package beam;

/**
 *
 * @author Kamran
 */
public class TestBeam {
    
   public static void main(String args[]) {
      //-----EXAMPLE 15.1-----\\ 
        //Define Joints
        Joint j1 = new Joint(0, 0);
        Joint j2 = new Joint(2, 0);
        Joint j3 = new Joint(4, 0); 
        //set support condition
        j1.setyRestrained(true);
        j1.setIsFySet(false);   // note when we apply restrains then we assume that load is taken by support
        j2.setyRestrained(true);
        j2.setIsFySet(false);
        
        // create members
        Member m1=new Member(j1,j2);
        Member m2=new Member(j2,j3);
        
        //create model
        Model model=new Model();
        model.getMemberAl().add(m1);
        model.getMemberAl().add(m2);
                
        model.jYLoads(j3, -5);
        Compute c=new Compute(model);
            
       //// -------------END OF EXAMPLE 15.1---------------\\\\\\\\ 
      
       //// -------------EXAMPLE 15.3---------------\\\\\\\\ 
    /*   
       Joint j1 = new Joint(0, 0);
        Joint j2 = new Joint(2, 0);
        Joint j3 = new Joint(4, 0); 
        //set support condition
        j1.setyRestrained(true);
        j1.setIsFySet(false);   // note when we apply restrains then we assume that load is taken by support
        j2.setyRestrained(true);
        j2.setIsFySet(false);
        j2.setyRestrained(true);
        j2.setIsFySet(false);
        
        // create members
        Member m1=new Member(j1,j2);
        Member m2=new Member(j2,j3);
        
        //create model
        Model model=new Model();
        model.getMemberAl().add(m1);
        model.getMemberAl().add(m2);
                
        model.jZLoads(j1, 4);
        model.jZLoads(j3, -4);
        Compute c=new Compute(model);
      */  
        //// ------------END OF EXAMPLE 15.3---------------\\\\\\\\
   }
    
}
