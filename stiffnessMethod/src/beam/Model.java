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
         * The following loop move the restrained joint at the end of jointAL
         */
        int k = jointAl.size() - 1;
        for (int i = 0; i < this.jointAl.size(); i++) {
            if (jointAl.get(i).isxRestrained() || jointAl.get(i).isyRestrained()) {
                //For swapping elements
                if (i == k) {
                    break;
                }
                Joint temp;
                temp = jointAl.get(i);
                jointAl.set(i, jointAl.get(k));
                jointAl.set(k, temp);

                i = i - 1;
                k--;
            }

        }
        Joint temp;
        temp = jointAl.get(jointAl.size() - 2);
        jointAl.set(jointAl.size() - 2, jointAl.get(jointAl.size() - 1));
        jointAl.set(jointAl.size() - 1, temp);

        /*
         * The following loop moves the join in up direction if it is restrained
         * in only one direction.
         * 
         */
        for (int i = 0; i < jointAl.size(); i++) {
            if ((i + 1) < jointAl.size()) {
                if ((!jointAl.get(i + 1).isxRestrained() && jointAl.get(i + 1).isyRestrained())
                        && (jointAl.get(i).isxRestrained() && jointAl.get(i).isyRestrained())
                        || (jointAl.get(i + 1).isxRestrained() && !jointAl.get(i + 1).isyRestrained())
                        && (jointAl.get(i).isxRestrained() && jointAl.get(i).isyRestrained())) {

                    Joint temp1;
                    temp1 = jointAl.get(i);
                    jointAl.set(i, jointAl.get(i+1));
                    jointAl.set(i+1, temp1);
                }
            }
        }

        /*The following loop gives joint numbers. each join has two 
         * two number representing displacement and force in x and y 
         * direction. In other word each joint has four quantities. 
         */
        int j = 1;
        for (int i = 0; i < jointAl.size(); i++) {
            jointAl.get(i).setxNo(j);
            jointAl.get(i).setyNo(j + 1);
            j = j + 2;
        }
        /*
         * The following segment checks if a joint has restrained in only one
         * direction then whether the unrestrained direction is assigned a 
         * lower number.
         */

        for (int i = 0; i < jointAl.size(); i++) {
            if ((jointAl.get(i).getxNo() < jointAl.get(i).getyNo())&&
                    jointAl.get(i).isxRestrained() && !jointAl.get(i).isyRestrained()){
                int temp2=jointAl.get(i).getxNo();
                jointAl.get(i).setxNo(jointAl.get(i).getyNo());
                jointAl.get(i).setyNo(temp2);
                        
            }
                     
         {
                
            }
        }

    }

    private void swap(Joint x, Joint y) {
        Joint temp = x;
        x = y;
        y = temp;
    }

    /*
     * Following two methods are written to apply loads on the truss
     * Note that loads are applied in unrestrained directions
     * the isFxSet and isFySet parameters are set true after the loads is
     * are applied
     */
    public void jXLoads(Joint j, double qx){
        j.setFx(qx);
        j.setIsFxSet(true);
         
    }
    public void jYLoads(Joint j,  double qy){
        j.setFy(qy);
        j.setIsFySet(true);
    }
}
