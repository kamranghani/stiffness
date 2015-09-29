package truss;

/**
 *
 * @author Kamran
 */
import Jama.Matrix;

public class Compute {

    private Model model;
    private int size = 0;
    //global stiffness matrix
    private double[][] sMat;
    //for finding displacement and unknow reaction (same term as in Hibbeler 8th addition page 552
    private double[][] K11;
    private double[][] K21;
    private double[][] K12;
    private double[][] K22;
    
    // matrix of known loads
    double[][] load;
    //matrix of know displacements
    double [][] disp;

    public Compute(Model mo) {
        this.model = mo;
        model.populateJointAl();
        size = model.getJointAl().size();
        System.out.println("size is=" + size);
        this.sMat = new double[size * 2][size * 2];  //multiply by 2 because of two degree of freedom     
        this.populateSMat(mo);
        //construct K11 and K22 matrix for load and displacement calculation
       // this.extractK11_K22(mo);
        this.extractK11_K21_K12_K22(mo);
        //calculate member forces
        this.calcUnknowns();

    }

    public void setSize(int s) {
        this.size = s;
    }

    public int getSize() {
        return size;
    }

    /**
     * the following method is written to construct the golbal stiffness matrix
     */
    private void populateSMat(Model mo) {
        //member stiffness matrix
        //double[][] mSMat=new double[4][4];
        for (int i = 0; i < mo.getMemberAl().size(); i++) {
            //construct member stiffness member      
            double[][] msmat;
            msmat = constructMSMat(mo.getMemberAl().get(i));
            //since we know the number of elements to be entered
            //and we also know the joint numbers of each member
            //therefore we will enter the values in global stiffness
            // matrix using the joint numbers of the member

            int nx = mo.getMemberAl().get(i).getN().getxNo(); //near end x
            int ny = mo.getMemberAl().get(i).getN().getyNo();
            int fx = mo.getMemberAl().get(i).getF().getxNo();
            int fy = mo.getMemberAl().get(i).getF().getyNo();

            sMat[nx - 1][nx - 1] += msmat[0][0];
            sMat[nx - 1][ny - 1] += msmat[0][1];
            sMat[nx - 1][fx - 1] += msmat[0][2];
            sMat[nx - 1][fy - 1] += msmat[0][3];
            sMat[ny - 1][nx - 1] += msmat[1][0];
            sMat[ny - 1][ny - 1] += msmat[1][1];
            sMat[ny - 1][fx - 1] += msmat[1][2];
            sMat[ny - 1][fy - 1] += msmat[1][3];
            sMat[fx - 1][nx - 1] += msmat[2][0];
            sMat[fx - 1][ny - 1] += msmat[2][1];
            sMat[fx - 1][fx - 1] += msmat[2][2];
            sMat[fx - 1][fy - 1] += msmat[2][3];
            sMat[fy - 1][nx - 1] += msmat[3][0];
            sMat[fy - 1][ny - 1] += msmat[3][1];
            sMat[fy - 1][fx - 1] += msmat[3][2];
            sMat[fy - 1][fy - 1] += msmat[3][3];



             printMAT(msmat);
            
             System.out.println("--------------------------------------");
             System.out.println("---------------Another member---------");
             System.out.println("--------------------------------------");
             //*/
        }
        printMAT(sMat);

        //  System.out.println("--------------------------------------");
        // System.out.println("---------------Another member---------");
        // System.out.println("--------------------------------------");

    }

    /**
     * for constructing member stiffness matrix
     *
     */
    public double[][] constructMSMat(Member m) {
        //for storing lambda values
        double lambda[] = cosine(m.getN().getX(), m.getF().getX(), m.getN().getY(), m.getF().getY());
        double[][] mSMat;
        mSMat = new double[4][4];
        //To reduce complexity, we avoided the loops. The following can
        // alse be entered using loops
        mSMat[0][0] = lambda[0] * lambda[0];
        mSMat[0][1] = lambda[0] * lambda[1];
        mSMat[0][2] = lambda[0] * (-lambda[0]);
        mSMat[0][3] = lambda[0] * (-lambda[1]);
        mSMat[1][0] = lambda[0] * lambda[1];
        mSMat[1][1] = lambda[1] * lambda[1];
        mSMat[1][2] = lambda[0] * (-lambda[1]);
        mSMat[1][3] = lambda[1] * (-lambda[1]);
        mSMat[2][0] = (-lambda[0]) * lambda[0];
        mSMat[2][1] = (-lambda[0]) * lambda[1];
        mSMat[2][2] = (-lambda[0]) * (-lambda[0]);
        mSMat[2][3] = (-lambda[0]) * (-lambda[1]);
        mSMat[3][0] = (-lambda[0]) * lambda[1];
        mSMat[3][1] = (-lambda[1]) * lambda[1];
        mSMat[3][2] = (-lambda[0]) * (-lambda[1]);
        mSMat[3][3] = (-lambda[1]) * (-lambda[1]);

        //divide each element by Length L
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                mSMat[i][j] = mSMat[i][j] / lambda[2];
            }
        }
        return mSMat;
    }

    /**
     * Method is used to calculate cosine called lambda in Hibbeler book.
     *
     * @param xn near joint x component
     * @param xf far joint x component
     * @param yn near joint y component
     * @param yf far joint y component
     * @return
     */
    public double[] cosine(double xn, double xf, double yn, double yf) {

        //length of member
        double length = Math.sqrt((xf - xn) * (xf - xn) + (yf - yn) * (yf - yn));
        double cosineX = (xf - xn) / length;
        double cosineY = (yf - yn) / length;

        double cosineAr[] = {cosineX, cosineY, length};//or call is lambdaX and lambdaY in accordance with hibbeler book.
        return cosineAr;
    }

    //for printing two dimensional array.
    public void printMAT(double[][] m) {
        for (int i = 0; i < m.length; i++) {
            for (int j = 0; j < m[i].length; j++) {
                System.out.printf("%1.3f", m[i][j]);
                System.out.print("  ");
            }
            System.out.println();
        }
    }

    //For calculating unknown displacements.
    /**
     * Compute the displacements cause by applied loads
     *
     * @param mo
     */
  /*  private void extractK11_K22(Model mo) {
        // count the no of loads which have been applied

        int count = 0;
        for (int i = 0; i < mo.getJointAl().size(); i++) {
            if (mo.getJointAl().get(i).isIsFxSet()) {
                count = count + 1;
            }
            if (mo.getJointAl().get(i).isIsFySet()) {
                count = count + 1;
            }
        }
        //Construct a load matrix
        double[][] load = new double[count][1];


        //based on no of known loads create the K11 and K12 matrices
        //If isIsFxSet then get the joint xNo for i and add values fri1
        //till 
        this.K11 = new double[count][count];
        this.K12 = new double[(mo.getJointAl().size() * 2) - count][ count];
        int k=0; //counter variable for K12
        for (int i = 0; i < mo.getJointAl().size(); i++) {
            int x = mo.getJointAl().get(i).getxNo() - 1;
            int y = mo.getJointAl().get(i).getyNo() - 1;
            if (mo.getJointAl().get(i).isIsFxSet()) {

                load[x][0] = mo.getJointAl().get(i).getFx(); //Note: Here might be a bug, if x is not as per Hibbeler or we have more than one roller support.
                for (int j = 0; j < count; j++) {
                    K11[x][j] = this.sMat[x][j];
                }
            } else {
                for (int j = 0; j < count; j++) {
                    K12[k][j] = this.sMat[x][j];
                    
                }
                k++;
            }

                if (mo.getJointAl().get(i).isIsFySet()) {

                    load[y][0] = mo.getJointAl().get(i).getFy();
                    for (int j = 0; j < count; j++) {
                        K11[y][j] = this.sMat[y][j];
                    }
                } else {
                    for (int j = 0; j < count; j++) {
                        K12[k][j] = this.sMat[y][j];
                        
                    }
                    k++;
                }
            
        }
        //System.out.println("-------K11 Matrix-----");
        //printMAT(K11);
        Matrix k11 = new Matrix(K11);
        Matrix q = new Matrix(load);
        Matrix d = k11.inverse().times(q);
        
        System.out.println("-------Displacements are-----");
        printMAT(d.getArray());
        printMAT(K12);
        
        System.out.println("-------reactions are-----");
        
        Matrix k12=new Matrix(K12);
        Matrix q1=k12.times(d);
        printMAT(q1.getArray());
    }
    
   /**
    * Method is used for calculating member forces after finding
    * joint displacements and support reactions
    */
    
    
    
    
    private void extractK11_K21_K12_K22(Model mo) {
        // count the no of unknow displacements i.e, the values for which isyRestrained() and iszRestrained()
        //has been set.

        int countR = 0;
        for (int i = 0; i < mo.getJointAl().size(); i++) {
            if (mo.getJointAl().get(i).isyRestrained()) {
                countR = countR + 1;
            }
            if (mo.getJointAl().get(i).isxRestrained()) {
                countR = countR + 1;
            }
        }

        //From countR find count i.e., the no of load applied. Note based on support condition
        //we will take loads equal to zero if not a load is not applied
        int count = mo.getJointAl().size() * 2 - countR;
        //Construct a load matrix
        load = new double[count][1];

        // We have Qk=K11 Du+K12 Dk and 
        //         Qu=K21 Du+K22 Dk    where subscript u =unknow and k=known
        //based on no of known loads create the K11 and K12 matrices
        //If isIsFzSet then get the joint zNo for i and add values and vice versa
        //
        this.K11 = new double[count][count];
        
        this.K12 = new double[count][(mo.getJointAl().size() * 2) - count];
        this.K21 = new double[(mo.getJointAl().size() * 2) - count][count];
        
        this.K22=new double[(mo.getJointAl().size() * 2) - count][(mo.getJointAl().size() * 2) - count];
        int k = 0; //temporary counter
        int m = 0; //temporary counter;
        
        for (int i=0;i<count;i++){
            for (int j=0;j<count;j++){
                K11[i][j]=this.sMat[i][j];
            }
        }
        
        for (int i=0;i<count;i++){
            for (int j=count;j<(count+countR);j++){
                K12[i][m]=this.sMat[i][j];
                m++;
            } 
            m=0;
            
    }
        for (int i=count;i<count+countR;i++){
            for (int j=0;j<count;j++){
                K21[m][j]=this.sMat[i][j];
            }
                           m++;
        }
         m=0;

        for (int i=count;i<count+countR;i++){
            for (int j=count;j<count+countR;j++){
                K22[k][m]=this.sMat[i][j];
                
                m++;
            }
            k++;
            m=0;
        }
        
        
        //for storing known loads
        for (int i = 0; i < mo.getJointAl().size(); i++) {
            int x = mo.getJointAl().get(i).getxNo() - 1;
            int y = mo.getJointAl().get(i).getyNo() - 1;
            if (mo.getJointAl().get(i).isIsFxSet()) {
                load[x][0] = mo.getJointAl().get(i).getFx();
            }
            
            if (mo.getJointAl().get(i).isIsFySet()) {

                load[y][0] = mo.getJointAl().get(i).getFy();
            }
        }
        
        /*
               
        for (int i = 0; i < mo.getJointAl().size(); i++) {
            int x = mo.getJointAl().get(i).getzNo() - 1;
            int y = mo.getJointAl().get(i).getyNo() - 1;
            
            if (mo.getJointAl().get(i).isIsFzSet()) {
                load[x][0] = mo.getJointAl().get(i).getFz();
                for (int j = 0; j < count; j++) {
                    K11[x][j] = this.sMat[x][j];
                }
                for (int j = count; j < (count + countR); j++) {
                    K12[x][m] = this.sMat[x][j];
                    m++;
                }
                m = 0;  //reset counter for K12 
            } else {
                for (int j = 0; j < countR; j++) {
                    K21[k][j] = this.sMat[x][j];

                }
                for (int j = count; j < (count + countR); j++) {
                    K22[k][m] = this.sMat[x][j];
                    m++;
                }
                m = 0;

                k++;
            }

            if (mo.getJointAl().get(i).isIsFySet()) {

                load[y][0] = mo.getJointAl().get(i).getFy();
                for (int j = 0; j < count; j++) {
                    K11[y][j] = this.sMat[y][j];
                }
                for (int j = count; j < (count + countR); j++) {
                    K12[y][m] = this.sMat[y][j];
                    m++;
                }
                m = 0;
            } else {
                for (int j = 0; j < count; j++) {
                    K21[k][j] = this.sMat[y][j];
                }
                for (int j = count; j < (count + countR); j++) {
                    K22[k][m] = this.sMat[y][j];
                    m++;
                }
                m = 0;
                k++;
            }

        }
*/    }
    
    public void calcUnknowns(){      
        
        System.out.println("-------K11 Matrix-----");
        printMAT(K11);
        System.out.println("-------K12 Matrix-----");
        printMAT(K12);
        System.out.println("-------K21 Matrix-----");
        printMAT(K21);
        System.out.println("-------K22 Matrix-----");
        printMAT(K22);
        
        //following loop is for populating the known displacement matrix .i.e.Dk
        int k=0;
        disp=new double [K12[0].length][1];
          for(int i=0;i<this.model.getJointAl().size();i++){
              if(this.model.getJointAl().get(i).isyDisplaced()){
                  this.disp[k][0]=this.model.getJointAl().get(i).getDy();
                  k++;
              }
              
              if(this.model.getJointAl().get(i).isxDisplaced()){
                  this.disp[k][0]=this.model.getJointAl().get(i).getDx();
                  k++;
              }
          }
          
         // printMAT(disp);
         Matrix k11 = new Matrix(K11);
        // System.out.println("K12 MATRIX");
         //printMAT(K12);
        Matrix k12 = new Matrix(K12);
        Matrix k21=new Matrix(K21);
        Matrix k22=new Matrix(K22);
         
         Matrix qk = new Matrix(load);
         Matrix dk=new Matrix(disp);
         System.out.println("displacements");
         printMAT(dk.getArray());
         
         Matrix du =((k11.inverse()).times(qk)).minus(((k11.inverse()).times(k12).times(dk))) ;  // Du=K11^1 *qk-k11^1*k12*dk (where k11^1 is inverse inverse of k11)
        
         System.out.println("-------Displacements are-----");
         printMAT(du.getArray());
         printMAT(K21);
        
         System.out.println("-------reactions are-----");
        
        //Matrix q1=k21.times(du);
         Matrix q1=(k21.times(du)).plus(k22.times(dk));  // qu=K21 Du+K22*Dk
         printMAT(q1.getArray());  
        // printMAT(load);
    }
    
    
}
