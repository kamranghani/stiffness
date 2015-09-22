/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package beam;


import java.util.ArrayList;
//import java.util.Comparator;
//import java.util.ListIterator;

/**
 *
 * @author Kamran This class is used to construct truss model. The model is
 * stored in an arrayList of member elements. each end.
 */
public class Model {
    //Array for storing member data

    private ArrayList<Member> memberAl = new ArrayList<Member>();
    //Array for storing joint data
    private ArrayList<Joint> jointAl = new ArrayList<Joint>();
    
    //
    private ArrayList loadAl=new ArrayList();

    //Adding member in arraylist
    public void addMember(Member m) {
        getMemberAl().add(m);

    }

    //for sorting member in arrayList based on x coordinates
    public ArrayList<Member> getMemberAl() {
        return memberAl;
    }

    public ArrayList<Joint> getJointAl() {
        return jointAl;
    }

    //populating jointAL from member data. 
    public void populateJointAl() {

        //jointAl.add(memberAl.get(0).getN());
        //jointAl.add(memberAl.get(0).getF());
        for (int i = 0; i < memberAl.size(); i++) {
            if (!(jointAl.contains(memberAl.get(i).getN()))) {
                jointAl.add(memberAl.get(i).getN());
            }
            if (!(jointAl.contains(memberAl.get(i).getF()))) {
                jointAl.add(memberAl.get(i).getF());
            }

        }
       
        
        /*
        * The following loops assigned lower numbers to unrestrained directions
        * of joints.
        */
        int jNo=1;
        for (int i=0;i<(this.jointAl.size());i++){
            if (!jointAl.get(i).isyRestrained()){
                jointAl.get(i).setyNo(jNo);
                jNo++;
            }
            if (!jointAl.get(i).iszRestrained()){
                jointAl.get(i).setzNo(jNo);
                jNo++;
                
            }
            
        }
        /*
        *After executing above loops the following loop assign higher nobs
        * to the restrained joints.
        */
        for (int i=0;i<(this.jointAl.size());i++){
            if (jointAl.get(i).isyRestrained()){
                jointAl.get(i).setyNo(jNo);
                jNo++;
            }
            if (jointAl.get(i).iszRestrained()){
                jointAl.get(i).setzNo(jNo);
                jNo++;
                
            }
            
        }
        
     
    }   

    /*
     * Following two methods are written to apply loads on the Beam
     * Note that loads are applied in unrestrained directions
     * the isFxSet and isFySet parameters are set true after the loads is
     * are applied
     */
    public void jZLoads(Joint j, double qz){
        j.setFz(qz);
        j.setIsFzSet(true);         
    }
    public void jYLoads(Joint j,  double qy){
        j.setFy(qy);
        j.setIsFySet(true);
    }
    
    public void jZDisp(Joint j, double dz){
        j.setDz(dz);
        j.setzDisplaced(true);         
    }
    
    public void jYDisp(Joint j,  double dy){
        j.setDy(dy);
        j.setyDisplaced(true);
    }
}
