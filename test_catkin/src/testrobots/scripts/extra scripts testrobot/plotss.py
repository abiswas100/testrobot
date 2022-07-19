
# #plot initial cov matrix P -----------------------------------------------------------------------------------------
# fig = plt.figure(figsize=(6, 6))
# im = plt.imshow(P, interpolation="none", cmap=plt.get_cmap('binary'))
# plt.title('Initial Covariance Matrix $P$')
# ylocs, ylabels = plt.yticks()
# # set the locations of the yticks
# plt.yticks(np.arange(10))
# # set the locations and labels of the yticks
# plt.yticks(np.arange(9),('$x$', '$y$', '$z$', '$\dot x$', '$\dot y$', '$\dot z$', '$\ddot x$', '$\ddot y$', '$\ddot z$'), fontsize=22)

# xlocs, xlabels = plt.xticks()
# # set the locations of the yticks
# plt.xticks(np.arange(7))
# # set the locations and labels of the yticks
# plt.xticks(np.arange(9),('$x$', '$y$', '$z$', '$\dot x$', '$\dot y$', '$\dot z$', '$\ddot x$', '$\ddot y$', '$\ddot z$'), fontsize=22)

# plt.xlim([-0.5,8.5])
# plt.ylim([8.5, -0.5])

# from mpl_toolkits.axes_grid1 import make_axes_locatable
# divider = make_axes_locatable(plt.gca())
# cax = divider.append_axes("right", "5%", pad="3%")
# plt.colorbar(im, cax=cax)


# plt.tight_layout()

# #-----------------------------------------------------------------------------------------------

# #plot measurement noise cov matrix R--------------------------------------------------------------------------------------

# fig = plt.figure(figsize=(4, 4))
# im = plt.imshow(R, interpolation="none", cmap=plt.get_cmap('binary'))
# plt.title('Measurement Noise Covariance Matrix $R$')
# ylocs, ylabels = plt.yticks()
# # set the locations of the yticks
# plt.yticks(np.arange(4))
# # set the locations and labels of the yticks
# plt.yticks(np.arange(3),('$x$', '$y$', '$z$'), fontsize=22)

# xlocs, xlabels = plt.xticks()
# # set the locations of the yticks
# plt.xticks(np.arange(4))
# # set the locations and labels of the yticks
# plt.xticks(np.arange(3),('$x$', '$y$', '$z$'), fontsize=22)

# plt.xlim([-0.5,2.5])
# plt.ylim([2.5, -0.5])

# from mpl_toolkits.axes_grid1 import make_axes_locatable
# divider = make_axes_locatable(plt.gca())
# cax = divider.append_axes("right", "5%", pad="3%")
# plt.colorbar(im, cax=cax)

# plt.tight_layout()

# #-------------------------------------------------------------------------------------------------

# # plot process noise covariance matrix Q--------------------------------------------------------------------
# fig = plt.figure(figsize=(6, 6))
# im = plt.imshow(Q, interpolation="none", cmap=plt.get_cmap('binary'))
# plt.title('Process Noise Covariance Matrix $Q$')
# ylocs, ylabels = plt.yticks()
# # set the locations of the yticks
# plt.yticks(np.arange(10))
# # set the locations and labels of the yticks
# plt.yticks(np.arange(9),('$x$', '$y$', '$z$', '$\dot x$', '$\dot y$', '$\dot z$', '$\ddot x$', '$\ddot y$', '$\ddot z$'), fontsize=22)

# xlocs, xlabels = plt.xticks()
# # set the locations of the yticks
# plt.xticks(np.arange(7))
# # set the locations and labels of the yticks
# plt.xticks(np.arange(9),('$x$', '$y$', '$z$', '$\dot x$', '$\dot y$', '$\dot z$', '$\ddot x$', '$\ddot y$', '$\ddot z$'), fontsize=22)

# plt.xlim([-0.5,8.5])
# plt.ylim([8.5, -0.5])

# from mpl_toolkits.axes_grid1 import make_axes_locatable
# divider = make_axes_locatable(plt.gca())
# cax = divider.append_axes("right", "5%", pad="3%")
# plt.colorbar(im, cax=cax)


# plt.tight_layout()

# #--------------------------------------------------------------------------------------------