# GSim Skeleton Definition - M. Kallmann 2006

KnSkeleton

name primo

ik leftarm lwrist
ik rightarm rwrist
ik leftleg lankle
ik rightleg rankle

skeleton

root hips
  { offset 0 91.5 0
      visgeo primitive
             capsule 8.2 7.4 6 20
             center 0 2 0
             orientation axis 1 0 0 ang 5
             color 40 52 96 255;
    channel XPos 0 free
    channel YPos 0 free
    channel ZPos 0 free
    channel Quat

    joint sacroiliac
    { offset 0 0 0 

      joint abdomen
      { offset 0 9.5474 0
        visgeo primitive
               capsule 7 8.5 7 20
               center 0 7 0
               color 180 175 45 255;

        channel Quat

        joint thorax
        { offset 0 14.165 0
          visgeo primitive
                 capsule 9.2 11.6 9 20
                 center 0 13.5 0
                                color 180 175 45 255;

          channel Quat

          joint vc7
          { offset 0 27.43 0

            joint neck
            { offset 0 9.54 0
              visgeo primitive
                     capsule 3 2.8 5.2 20
                     center 0 1.557 0.3
                     orientation axis 1 0 0 ang 10.4796
                     color 240 200 155 255;

              channel Quat

              joint head
              { offset 0 3.114 0
                visgeo primitive
                       capsule 6.6 7.6 4.6 20
                       center 0 6.5 4
                       color 240 200 155 255;

                channel Quat
              }
            }
          }

          joint rcollar
          { offset -8 24.43 0
            visgeo primitive
                   capsule 5.5 4.8 7 20
                   center -3.5 0 0
                   orientation axis 0 0 1 ang 90
                   color 180 175 45 255;


            joint rshoulder
            { offset -10.45 0 0
              visgeo primitive
                     capsule 5 3.5 13.75 20
                     center -13.75 0 0
                     orientation axis 0 0 1 ang 90
                                    color 180 175 45 255;

              channel Swing lim 110 150
              channel Twist 0 lim -90 120
              prerot axis 0 -1 0 ang 90
              postrot axis 0 1 0 ang 90

              joint relbow
              { offset -27.5 0 0
                visgeo primitive
                       capsule 3.4 3.3 13.35 20
                       center -13.35 0 0
                       orientation axis 0 0 1 ang 90
                       color 240 200 155 255;

                euler ZY
                channel YRot 0 lim 0 160
                channel ZRot 0 lim -120 45
                prerot axis 0 -1 0 ang 90
                postrot axis 0 1 0 ang 90

                joint rwrist
                { offset -26.7 0 0
                    visgeo primitive
                           box 7.4 3.8 1.2 20
                           center -10.7 0 0 
                           color 240 200 155 255;

                  channel Swing lim 50 90
                  prerot axis 0 -1 0 ang 90
                  postrot axis 0 1 0 ang 90

                  joint rthumb
                  { offset -14 3 0
                  }

                  joint rpinky
                  { offset -14 -3 0
                  }

                }
              }
            }
          }

          joint lcollar
          { offset 8 24.43 0
            visgeo primitive
                   capsule 5.5 4.8 7 20
                   center 3.5 0 0
                   orientation axis 0 0 -1 ang 90
                   color 180 175 45 255;


            joint lshoulder
            { offset 10.45 0 0
              visgeo primitive
                     capsule 5 3.5 13.75 20
                     center 13.75 0 0
                     orientation axis 0 0 -1 ang 90
                                    color 180 175 45 255;

              channel Swing lim 110 150
              channel Twist 0 lim -90 120
              prerot axis 0 1 0 ang 90
              postrot axis 0 -1 0 ang 90

              joint lelbow
              { offset 27.5 0 0
                visgeo primitive
                       capsule 3.4 3.3 13.35 20
                       center 13.35 0 0
                       orientation axis 0 0 -1 ang 90
                       color 240 200 155 255;

                euler ZY
                channel YRot 0 lim -160 0
                channel ZRot 0 lim -45 120
                prerot axis 0 1 0 ang 90
                postrot axis 0 -1 0 ang 90

                joint lwrist
                { offset 26.7 0 0
                    visgeo primitive
                           box 7.4 3.8 1.2 20
                           center 10.7 0 0 
                           color 240 200 155 255;

                  channel Swing lim 50 90
                  prerot axis 0 1 0 ang 90
                  postrot axis 0 -1 0 ang 90

                  joint lthumb
                  { offset 11 3 0
                  }

                  joint lpinky
                  { offset 14 -3 0
                  }
                }
              }
            }
          }
        }
      }

      joint rhip
      { offset -8 -4.16 0
        visgeo primitive
               capsule 7 5 21 20
               center 0 -21 0
               orientation axis -1 0 0 ang 180
               material amb 51 51 51 255 dif 40 52 96 255 spe 0 0 0 255 emi 0 0 0 255 shi 0;

        channel Swing lim 65 120
        channel Twist 0 lim -30 100
        prerot axis 0.57735 0.57735 -0.57735 ang 120
        postrot axis -0.57735 -0.57735 0.57735 ang 120

        joint rknee
        { offset 0 -40.735 0
          visgeo primitive
                 capsule 5 4 19 20
                 center 0 -20 0
                 orientation axis -1 0 0 ang 180
                 material amb 51 51 51 255 dif 40 52 96 255 spe 0 0 0 255 emi 0 0 0 255 shi 0;

          euler ZY
          channel YRot 0 lim 0 160
          channel ZRot 0 lim -55 55
          prerot axis 0.57735 0.57735 -0.57735 ang 120
          postrot axis -0.57735 -0.57735 0.57735 ang 120

          joint rankle
          { offset 0 -38 0
            visgeo primitive
                   box 4 2 12 20
                   center 0 -5.5 6.5
                   material amb 51 51 51 255 dif 111 65 43 255 spe 0 0 0 255 emi 0 0 0 255 shi 0;

            channel Swing lim 35 60
            prerot axis 0.57735 0.57735 -0.57735 ang 120
            postrot axis -0.57735 -0.57735 0.57735 ang 120
            
            joint rtoes
             { offset 5 0 0
             }
          }
        }
      }

      joint lhip
      { offset 8 -4.16 0
        visgeo primitive
               capsule 7 5 21 20
               center 0 -21 0
               orientation axis -1 0 0 ang 180
               material amb 51 51 51 255 dif 40 52 96 255 spe 0 0 0 255 emi 0 0 0 255 shi 0;

        channel Swing lim 65 120
        channel Twist 0 lim -100 30
        prerot axis 0.57735 0.57735 -0.57735 ang 120
        postrot axis -0.57735 -0.57735 0.57735 ang 120

        joint lknee
        { offset 0 -40.735 0
          visgeo primitive
                 capsule 5 4 19 20
                 center 0 -20 0
                 orientation axis -1 0 0 ang 180
                 material amb 51 51 51 255 dif 40 52 96 255 spe 0 0 0 255 emi 0 0 0 255 shi 0;

          euler ZY
          channel YRot 0 lim 0 160
          channel ZRot 0 lim -55 55
          prerot axis 0.57735 0.57735 -0.57735 ang 120
          postrot axis -0.57735 -0.57735 0.57735 ang 120

          joint lankle
          { offset 0 -38 0
            visgeo primitive
                   box 4 2 12 21
                   center 0 -5.5 6.5
                   material amb 51 51 51 255 dif 111 65 43 255 spe 0 0 0 255 emi 0 0 0 255 shi 0;

            channel Swing lim 35 60
            prerot axis 0.57735 0.57735 -0.57735 ang 120
            postrot axis -0.57735 -0.57735 0.57735 ang 120
            
            joint ltoes
             { offset 5 0 0
             }

          }
        }
      }
    }
  }

end
