# SHODH Product Video Guide

## Video Specifications

### Technical Requirements
- **Resolution**: 1920x1080 (Full HD) or higher
- **Aspect Ratio**: 16:9
- **Format**: MP4 (H.264) and WebM for web compatibility
- **Duration**: 60-90 seconds (maximum 2 minutes)
- **File Size**: Under 50MB for fast loading
- **Frame Rate**: 30fps or 60fps
- **Audio**: Optional, but recommended with clear narration

### File Locations
Place your video files here:
- `public/demo-video.mp4` (required)
- `public/demo-video.webm` (optional, for better browser support)
- `public/video-poster.jpg` (thumbnail shown before play)

---

## What to Record

### Recommended Structure (60-90 seconds total)

#### 1. **Opening Shot** (5-10 seconds)
- Show the SHODH logo and tagline
- Text overlay: "Enterprise RAG with 97% Citation Accuracy"
- Smooth fade into the interface

#### 2. **Document Upload** (10-15 seconds)
- Drag and drop multiple documents (PDFs, Word docs, code files)
- Show the upload progress
- Highlight supported file types

#### 3. **Search & Query** (20-25 seconds)
- Type a complex question in the search bar
- Show the RAG system processing
- Display search results with:
  - Relevant document chunks
  - **Highlighted citations** (this is key!)
  - Confidence scores

#### 4. **Citation Accuracy Demo** (15-20 seconds)
- Click on a citation
- Show it opening the exact source document
- Highlight the referenced text
- This demonstrates the "97% citation accuracy"

#### 5. **Key Features** (10-15 seconds)
Quick montage showing:
- Knowledge graph visualization
- Multi-document comparison
- Code intelligence (if applicable)
- Real-time collaboration

#### 6. **Closing** (5-10 seconds)
- Show the dashboard with stats/metrics
- End with SHODH logo and CTA: "Get Started Today"
- Website URL: shodh.ai

---

## Recording Tips

### Software Recommendations
- **Screen Recording**:
  - OBS Studio (Free, professional)
  - Loom (Easy, cloud-based)
  - Camtasia (Paid, feature-rich)

### Recording Checklist
- [ ] Close unnecessary apps/notifications
- [ ] Use a clean, professional demo account
- [ ] Prepare sample documents beforehand
- [ ] Use smooth mouse movements (not jerky)
- [ ] Zoom in on important UI elements
- [ ] Add subtle background music (royalty-free)
- [ ] Include text overlays for key features

### Visual Style
- **Clean UI**: Hide personal info, use demo data
- **Smooth Transitions**: Fade between sections
- **Highlight Key Areas**: Use zoom or circles to draw attention
- **Professional Pace**: Not too fast, not too slow
- **Text Overlays**: Add feature names and benefits

---

## Narration Script (Optional)

> "Welcome to SHODH - the enterprise RAG platform with 97% citation accuracy.
>
> Simply upload your documents... ask complex questions... and get instant answers with verifiable sources.
>
> Every answer includes precise citations, linking directly to source material.
>
> Run entirely on your infrastructure. No API lock-in. Predictable costs.
>
> SHODH - Enterprise Document Intelligence. Get started today at shodh.ai"

---

## Alternative: Product Screenshots

If video is too time-consuming, create a slideshow with:
1. Upload interface screenshot
2. Query results with citations
3. Citation detail view
4. Knowledge graph
5. Dashboard/metrics

Place images in `public/screenshots/` and we'll create an auto-playing carousel.

---

## After Creating the Video

1. **Optimize the video**:
   ```bash
   # Compress MP4 (using ffmpeg)
   ffmpeg -i original.mp4 -c:v libx264 -crf 23 -preset slow -c:a aac -b:a 128k demo-video.mp4

   # Create WebM version
   ffmpeg -i demo-video.mp4 -c:v libvpx-vp9 -crf 30 -b:v 0 demo-video.webm

   # Create poster image (thumbnail)
   ffmpeg -i demo-video.mp4 -ss 00:00:03 -vframes 1 video-poster.jpg
   ```

2. **Place files in `public/` folder**
3. **Test loading speed** - should load in < 3 seconds

---

## Examples of Good Product Videos

Reference these for inspiration:
- Linear.app - Product demos
- Notion - Feature showcases
- Stripe - Technical product videos
- Vercel - Fast-paced feature highlights

Keep it fast, professional, and focused on the value proposition!
